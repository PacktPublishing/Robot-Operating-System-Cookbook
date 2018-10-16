#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include <opencv_candidate/feature2d.hpp>

#include <opencv_candidate_reconst3d/reconst3d.hpp>

using namespace std;
using namespace cv;

static inline
float tvecNorm(const Mat& Rt)
{
    return norm(Rt(Rect(3,0,1,3)));
}

static inline
float rvecNormDegrees(const Mat& Rt)
{
    Mat rvec;
    Rodrigues(Rt(Rect(0,0,3,3)), rvec);
    return norm(rvec) * 180. / CV_PI;
}

ArbitraryCaptureServer::TrajectorySegment::TrajectorySegment() :  representFrameIndex(-1), isFinalized(false)
{}

void ArbitraryCaptureServer::TrajectorySegment::push(const cv::Ptr<cv::OdometryFrame>& frame, const cv::Mat& pose,
                                               const cv::Mat& objectMask, const cv::Mat& tableMask,
                                               const cv::Vec4f& tableCoeff, const cv::Mat& bgrImage)
{
    frames.push_back(frame);
    poses.push_back(pose);
    objectMasks.push_back(objectMask);
    tableMasks.push_back(tableMask);
    tableCoeffs.push_back(tableCoeff);
    bgrImages.push_back(bgrImage);
}

ArbitraryCaptureServer::Feature2dEdge::Feature2dEdge(int srcSegmentIndex, int srcFrameIndex,
                                               int dstSegmentIndex, int dstFrameIndex,
                                               int inliersCount, const Mat& Rt) :
    srcSegmentIndex(srcSegmentIndex),
    srcFrameIndex(srcFrameIndex),
    dstSegmentIndex(dstSegmentIndex),
    dstFrameIndex(dstFrameIndex),
    inliersCount(inliersCount),
    Rt(Rt)
{}

ArbitraryCaptureServer::FramePushOutput::FramePushOutput() : isKeyframe(false)
{}

ArbitraryCaptureServer::ArbitraryCaptureServer() :
    minTranslationDiff(DEFAULT_MIN_TRANSLATION_DIFF()),
    minRotationDiff(DEFAULT_MIN_ROTATION_DIFF()),
    minObjectSize(DEFAULT_MIN_OBJECT_SIZE),
    isInitialied(false),
    isFinalized(false)
{}

void ArbitraryCaptureServer::initialize(const cv::Size& frameResolution)
{
    reset();

    CV_Assert(!cameraMatrix.empty());
    CV_Assert(cameraMatrix.type() == CV_32FC1);
    CV_Assert(cameraMatrix.size() == Size(3,3));

    normalsComputer = new RgbdNormals(frameResolution.height, frameResolution.width, CV_32FC1, cameraMatrix); // inner
    if(tableMasker.empty())
        tableMasker = new TableMasker();
    tableMasker->set("cameraMatrix", cameraMatrix);

    if(odometry.empty())
        odometry = new RgbdOdometry();
    odometry->set("cameraMatrix", cameraMatrix);

    if(featureComputer.empty())
        //featureComputer = new feature2d::AffineAdaptedFeature2D(new FastFeatureDetector(), new FREAK());
        featureComputer = new feature2d::AffineAdaptedFeature2D(new SURF());

    if(feature2dPoseEstimator.empty())
        feature2dPoseEstimator = new Feature2dPoseEstimator();
    feature2dPoseEstimator->set("cameraMatrix", cameraMatrix);

    isInitialied = true;
}

void ArbitraryCaptureServer::reset()
{
    prevTableMask.release();
    trajectorySegments.clear();
    feature2dEdges.clear();

    isFinalized = false;
}

Ptr<ArbitraryCaptureServer::TrajectorySegment> ArbitraryCaptureServer::getActiveSegment() const
{
    if(trajectorySegments.empty())
        return 0;

    Ptr<ArbitraryCaptureServer::TrajectorySegment> lastSegment = *trajectorySegments.rbegin();

    if(lastSegment->isFinalized)
        return 0;

    CV_Assert(!lastSegment->frames.empty());

    return lastSegment;
}

void ArbitraryCaptureServer::estimateFeatures2dEdges(int srcSegmentIndex, int srcFrameIndex,
                                                     const vector<KeyPoint>& srcKeypoints, const Mat& srcDescriptors, const Mat& srcCloud,
                                                     std::vector<Feature2dEdge>& edges) const
{
    edges.clear();
    for(size_t segmentIndex = 0; segmentIndex < trajectorySegments.size(); segmentIndex++)
    {
        int representFrameIndex = trajectorySegments[segmentIndex]->representFrameIndex;
        const vector<KeyPoint>& dstKeypoints = trajectorySegments[segmentIndex]->representFrameKeypoints;
        const Mat& dstDescriptors = trajectorySegments[segmentIndex]->representFrameDescriptors;
        Mat dstCloud;// = trajectorySegments[segmentIndex]->frames[representFrameIndex]->pyramidCloud[0];
        depthTo3d(trajectorySegments[segmentIndex]->frames[representFrameIndex]->depth, cameraMatrix, dstCloud);

        vector<DMatch> matches;
        cv::Mat Rt = (*feature2dPoseEstimator)(srcKeypoints, srcDescriptors, srcCloud,
                                               dstKeypoints, dstDescriptors, dstCloud, &matches);

        if(Rt.empty())
            continue;

        if(!Rt.empty())
            edges.push_back(Feature2dEdge(srcSegmentIndex, srcFrameIndex, segmentIndex,
                                          representFrameIndex, matches.size(), Rt));
    }
}

void ArbitraryCaptureServer::finalizeLastSegment()
{
    if(trajectorySegments.empty() || (*trajectorySegments.rbegin())->isFinalized)
        return;

    Ptr<TrajectorySegment> segment = *trajectorySegments.rbegin();

    // check frames count in the last segment
    const int minSegmentFramesCount = 4;
    if(static_cast<int>(segment->frames.size()) < minSegmentFramesCount)
    {
        // clear the last segment
        int segmentIndex = static_cast<int>(trajectorySegments.size()) - 1;

        trajectorySegments.resize(segmentIndex);

        std::vector<Feature2dEdge>::const_reverse_iterator it = feature2dEdges.rbegin(), rend = feature2dEdges.rend();
        int segmentEdgesCount = 0;
        for(; it != rend; ++it)
        {
            if(it->srcSegmentIndex == segmentIndex)
                segmentEdgesCount++;
            else
                break;
        }
        CV_Assert(segmentEdgesCount <= 1); // for the current approach

        feature2dEdges.resize(feature2dEdges.size() - segmentEdgesCount);
    }

    if(trajectorySegments.empty() && prevTableMask.empty())
    {
        // the table track was lost
        finalize();
        return;
    }

    // choose representative frame (it has large plane mask and a normal directed to the camera)
    CV_Assert(segment->frames.size() == segment->tableCoeffs.size());
    vector<float> polarAngles(segment->frames.size());
    vector<int> areas(segment->frames.size());
    float minPolarAngle = FLT_MAX;
    float maxPolarAngle = -1.f;
    float minArea = FLT_MAX;
    float maxArea = -1.f;
    for(size_t i = 0; i < segment->frames.size(); i++)
    {
        float area = cv::countNonZero(segment->tableMasks[i]);
        minArea = std::min(minArea, area);
        maxArea = std::max(maxArea, area);

        const Vec4f& coeffs = segment->tableCoeffs[i];
        float polarAngle = std::acos(-coeffs[2]);
        minPolarAngle = std::min(minPolarAngle, polarAngle);
        maxPolarAngle = std::max(maxPolarAngle, polarAngle);

        areas[i] = area;
        polarAngles[i] = polarAngle;
    }

    float areaScale = maxArea - minArea;
    areaScale = areaScale > FLT_EPSILON ? 1.f/areaScale : 0.;

    float polarAngleScale = maxPolarAngle - minPolarAngle;
    polarAngleScale = polarAngleScale > FLT_EPSILON ? 1.f/polarAngleScale : 0.;

    float maxRepresentRate = -1.f;
    int representIndex = -1;
    for(size_t i = 0; i < areas.size(); i++)
    {
        float representRate = (areas[i] - minArea) * areaScale + (maxPolarAngle - polarAngles[i]) * polarAngleScale;
        if(representRate > maxRepresentRate)
        {
            maxRepresentRate = representRate;
            representIndex = i;
        }
    }

    segment->representFrameIndex = representIndex;

    // compute features for the representative frame
    const Mat& image = segment->frames[representIndex]->image;
    Mat mask = segment->tableMasks[representIndex] | segment->objectMasks[representIndex];
    (*featureComputer)(image, mask, segment->representFrameKeypoints, segment->representFrameDescriptors);

    segment->isFinalized = true;
}

cv::Ptr<ArbitraryCaptureServer::TrajectorySegment> ArbitraryCaptureServer::createNewSegment()
{
    if(!trajectorySegments.empty())
    {
        CV_Assert((*trajectorySegments.rbegin())->isFinalized);
    }

    cv::Ptr<TrajectorySegment> segment = new TrajectorySegment();
    trajectorySegments.push_back(segment);

    return segment;
}

ArbitraryCaptureServer::FramePushOutput ArbitraryCaptureServer::push(const cv::Mat& image, const cv::Mat& depth, int frameID)
{
    FramePushOutput pushOutput;

    // check input data and currect state
    CV_Assert(isInitialied);
    CV_Assert(!isFinalized);

    CV_Assert(!normalsComputer.empty());
    CV_Assert(!tableMasker.empty());
    CV_Assert(!odometry.empty());
    CV_Assert(!featureComputer.empty());
    CV_Assert(!feature2dPoseEstimator.empty());

    if(image.empty() || depth.empty())
    {
        cout << "Warning: Empty frame " << frameID << endl;
        return pushOutput;
    }

    CV_Assert(image.size() == depth.size());
    CV_Assert(depth.type() == CV_32FC1);

    // precompute needed frame data
    Mat grayImage, cloud, normals;
    Mat tableMask, objectMask;
    Vec4f planeCoeffs;

    cvtColor(image, grayImage, CV_BGR2GRAY);
    depthTo3d(depth, cameraMatrix, cloud);
    (*normalsComputer)(cloud, normals);

    // find/compute previous table mask
    // (we can use empty previous table mask only for the first frame of dataset)
    vector<Feature2dEdge> edges;
    int bestEdgeIndex = -1;
    if(!trajectorySegments.empty() && getActiveSegment() == 0)
    {
        vector<KeyPoint> keypoints;
        Mat descriptors;
        (*featureComputer)(grayImage, Mat(), keypoints, descriptors);

        estimateFeatures2dEdges(trajectorySegments.size(), 0,
                                keypoints, descriptors, cloud, edges);

        if(edges.empty())
        {
            prevTableMask.release();
            cout << "Warning: can not match with any represent frame of all segments" << endl;
            return pushOutput;
        }

        int maxInliersCount = 0;
        for(size_t i = 0; i < edges.size(); i++)
        {
            if(edges[i].inliersCount > maxInliersCount)
            {
                bestEdgeIndex = i;
                maxInliersCount = edges[i].inliersCount;
            }
        }
        CV_Assert(bestEdgeIndex >= 0);

        if(prevTableMask.empty()) // TODO maybe get prevTableMask by warping although it's still valid
        {
            Mat warpedMask;
            const Ptr<RgbdFrame>& dstFrame = trajectorySegments[edges[bestEdgeIndex].dstSegmentIndex]->frames[edges[bestEdgeIndex].dstFrameIndex];
            const Mat& dstMask = trajectorySegments[edges[bestEdgeIndex].dstSegmentIndex]->tableMasks[edges[bestEdgeIndex].dstFrameIndex];
            const Mat& dstDepth = dstFrame->depth;
            warpFrame(dstMask, dstDepth, Mat(), edges[bestEdgeIndex].Rt.inv(DECOMP_SVD), cameraMatrix, Mat(), warpedMask);

            morphologyEx(warpedMask, warpedMask, MORPH_CLOSE, Mat(), Point(-1,-1), 7);

            int area = countNonZero(warpedMask);
            double areaPart = tableMasker->get<double>("minTablePart");
            if(area < image.total() * areaPart)
            {
                //finalizeLastSegment(); - it has to be already finalized
                return pushOutput;
            }
            prevTableMask = warpedMask;
        }
    }

    // find table mask in the current frame (using check of overlapping with the previous table mask)
    bool isTableMaskOk = (*tableMasker)(cloud, normals, prevTableMask, tableMask, objectMask, &planeCoeffs);

    if(!isTableMaskOk)
    {
        //we lost the table track:
        // stop the capture if the table masker fails on the first frame of the data,
        // otherwise finalize the last segment construction and
        // don't use the current frame at all

        if(trajectorySegments.empty())
        {
            finalize();

            cout << "Error: we suppose now that for the first frame of a dataset the table mask has to be detected correctly!" << endl;
            return pushOutput;
        }

        prevTableMask.release();
        finalizeLastSegment();

        cout << "Warning: bad table mask for the frame " << frameID << endl;
        return pushOutput;
    }

    prevTableMask = tableMask;

    if(countNonZero(objectMask) < minObjectSize)
    {
        // we lost the object track (but keep the table track):
        // finalize the last segment construction,
        // don't use the current frame for the futher model reconstruction but keep its table mask for the table tracking

        finalizeLastSegment();

        cout << "Warning: there is no object in the frame " << frameID << endl;
        return pushOutput;
    }

    // We know both table and object mask here.
    // If it's the first frame of the new segment we also know its transformation to the representative frames of some other segments

    Mat pose;
    Ptr<OdometryFrame> currFrame = new OdometryFrame(grayImage, depth, tableMask | objectMask, normals, frameID);
    if(trajectorySegments.empty())
    {
        // it's just a beginning of the trajectory construction
        pose = Mat::eye(4,4,CV_64FC1);
    }
    else
    {
        if(!edges.empty())
        {
            // it's the beggining of a new segment and we already estimated transformations from features2d
            Mat dstPose = trajectorySegments[edges[bestEdgeIndex].dstSegmentIndex]->poses[edges[bestEdgeIndex].dstFrameIndex];
            pose = dstPose * edges[bestEdgeIndex].Rt;
        }
        else
        {
            // we continue active segment construction
            Ptr<TrajectorySegment> segment = getActiveSegment();
            Mat Rt;
            bool isOdometryOk = odometry->compute(currFrame, segment->lastFrame, Rt);
            if(!isOdometryOk)
            {
                // we stop to construct the segment
                finalizeLastSegment();
                // TODO maybe try the current frame as the first frame of new segment
                return pushOutput;
            }

            pose = segment->lastPose * Rt;
        }
    }

    pushOutput.pose = pose;

    // find out if the frame is a keyframe?
    Ptr<TrajectorySegment> segment = getActiveSegment();
    if(segment.empty())
    {
        segment = createNewSegment();
        pushOutput.isKeyframe = true;
    }
    else
    {
        // check how far new frame from the last keyframe
        Mat prevKeyframePose = *segment->poses.rbegin();
        Mat poseDiff = prevKeyframePose.inv(DECOMP_SVD) * pose;

        float tnorm = tvecNorm(poseDiff);
        float rnorm = rvecNormDegrees(poseDiff);

        if((tnorm >= minTranslationDiff || rnorm >= minRotationDiff))
        {
            cout << "keyframe ID " << frameID << endl;
            pushOutput.isKeyframe = true;
        }
    }

    static int count = 0;
    if(pushOutput.isKeyframe)
    {
        segment->push(currFrame, pose, objectMask, tableMask, planeCoeffs, image);
        count++;
        cout << "Keyframes count " << count << std::endl;
    }

    if(bestEdgeIndex >= 0)
    {
        feature2dEdges.push_back(edges[bestEdgeIndex]);
    }

    segment->lastFrame = currFrame;
    segment->lastPose = pose;

    return pushOutput;
}

cv::Ptr<TrajectoryFrames> ArbitraryCaptureServer::finalize()
{
    Ptr<TrajectoryFrames> arbTrajectoryFrames = new TrajectoryFrames();

    arbTrajectoryFrames->resumeFrameState = TrajectoryFrames::KEYFRAME;

    int totalFrameIndex = 0;
    vector<int> segmentStartIndices(trajectorySegments.size());
    for(size_t segmentIndex = 0; segmentIndex < trajectorySegments.size(); segmentIndex++)
    {
        segmentStartIndices[segmentIndex] = totalFrameIndex;
        const Ptr<TrajectorySegment>& segment = trajectorySegments[segmentIndex];
        for(size_t frameIndex = 0; frameIndex < segment->frames.size(); frameIndex++)
        {
            const Ptr<OdometryFrame> odomFrame = segment->frames[frameIndex];
            Ptr<RgbdFrame> rgbdFrame = new RgbdFrame(segment->bgrImages[frameIndex], odomFrame->depth,
                                                     odomFrame->mask, odomFrame->normals, odomFrame->ID);
            const Mat& pose = segment->poses[frameIndex];
            const Mat& objectMask = segment->objectMasks[frameIndex];
            arbTrajectoryFrames->push(rgbdFrame, pose, objectMask, TrajectoryFrames::KEYFRAME);

            if(frameIndex > 0)
                arbTrajectoryFrames->keyframePosesLinks.push_back(PosesLink(totalFrameIndex, totalFrameIndex-1));

            totalFrameIndex++;
        }
    }

    for(size_t edgeIndex = 0; edgeIndex < feature2dEdges.size(); edgeIndex++)
    {
        const Feature2dEdge& edge = feature2dEdges[edgeIndex];
        int srcIndex = segmentStartIndices[edge.srcSegmentIndex] + edge.srcFrameIndex;
        int dstIndex = segmentStartIndices[edge.dstSegmentIndex] + edge.dstFrameIndex;

        arbTrajectoryFrames->keyframePosesLinks.push_back(PosesLink(srcIndex, dstIndex, edge.Rt));
    }

    // Try to set loop closure edge
    {
        Ptr<RgbdFrame> firstFrame, lastFrame;

        firstFrame = *(arbTrajectoryFrames->frames.begin());
        lastFrame = *(arbTrajectoryFrames->frames.rbegin());

        Mat firstGray, lastGray;
        cvtColor(firstFrame->image, firstGray, CV_BGR2GRAY);
        cvtColor(lastFrame->image, lastGray, CV_BGR2GRAY);

        vector<KeyPoint> firstKeypoints, lastKeypoints;
        Mat firstDescriptors, lastDescriptors;
        (*featureComputer)(firstGray, firstFrame->mask, firstKeypoints, firstDescriptors);
        (*featureComputer)(lastGray, lastFrame->mask, lastKeypoints, lastDescriptors);

        Mat firstCloud, lastCloud;
        depthTo3d(firstFrame->depth, cameraMatrix, firstCloud);
        depthTo3d(lastFrame->depth, cameraMatrix, lastCloud);
        vector<DMatch> matches;
        Mat Rt = (*feature2dPoseEstimator)(firstKeypoints, firstDescriptors, firstCloud,
                                           lastKeypoints, lastDescriptors, lastCloud, &matches);
        if(!Rt.empty() &&
           static_cast<int>(matches.size()) > feature2dPoseEstimator->get<int>("minInliersCount"))
        {
            arbTrajectoryFrames->keyframePosesLinks.push_back(PosesLink(0, arbTrajectoryFrames->frames.size() - 1, Rt));
        }
        cout << "Inliers count between the first and the last frames: " << matches.size() << endl;
    }

    isFinalized = true;

    return arbTrajectoryFrames;
}
