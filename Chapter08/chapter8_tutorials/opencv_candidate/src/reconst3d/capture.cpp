#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv_candidate_reconst3d/reconst3d.hpp>
#include <iomanip>

using namespace std;
using namespace cv;

const char checkDataMessage[] = "Please check the data! Sequential frames have to be close to each other (location and color). "
                                "The first and one of the last frames also have to be "
                                "really taken from close camera positions and the close lighting conditions.";

// get norms
//
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

//
static
float computeInliersRatio(const Ptr<OdometryFrame>& srcFrame,
                          const Ptr<OdometryFrame>& dstFrame,
                          const Mat& Rt, const Mat& cameraMatrix,
                          int maxColorDiff=CircularCaptureServer::DEFAULT_MAX_CORRESP_COLOR_DIFF,
                          float maxDepthDiff=CircularCaptureServer::DEFAULT_MAX_CORRESP_DEPTH_DIFF())
{
    Mat warpedSrcImage, warpedSrcDepth, warpedSrcMask;
    warpFrame(srcFrame->image, srcFrame->depth, srcFrame->mask,
              Rt, cameraMatrix, Mat(), warpedSrcImage, &warpedSrcDepth, &warpedSrcMask);

    int inliersCount = countNonZero(dstFrame->mask & warpedSrcMask &
                                    (cv::abs(dstFrame->image - warpedSrcImage) < maxColorDiff) &
                                    (cv::abs(dstFrame->depth - warpedSrcDepth) < maxDepthDiff));

    int intersectSize = countNonZero(dstFrame->mask & warpedSrcMask);
    return intersectSize ? static_cast<float>(inliersCount) / intersectSize : 0;
}

//---------------------------------------------------------------------------------------------------------------------------
PosesLink::PosesLink(int srcIndex, int dstIndex, const Mat& Rt)
    : srcIndex(srcIndex), dstIndex(dstIndex), Rt(Rt)
{}

void TrajectoryFrames::push(const Ptr<RgbdFrame>& frame, const Mat& pose, const Mat& objectMask, int state)
{
    CV_Assert(frame);
    CV_Assert(!pose.empty() && pose.size() == Size(4,4) && pose.type() == CV_64FC1);

    Ptr<RgbdFrame> rgbdFrame = new RgbdFrame();
    *rgbdFrame = *frame;
    frames.push_back(rgbdFrame);
    poses.push_back(pose);
    objectMasks.push_back(objectMask);
    frameStates.push_back(state);
}

void TrajectoryFrames::clear()
{
    frames.clear();
    poses.clear();
    frameStates.clear();
    keyframePosesLinks.clear();
}

void TrajectoryFrames::save(const std::string& dirname) const
{
    cout << "TrajectoryFrames::save" << endl;
    for(size_t i = 0; i < frames.size(); i++)
    {
        const Ptr<RgbdFrame> frame = frames[i];

        stringstream imageIndex;
        imageIndex << setfill('0') << setw(5) << frame->ID;

        imwrite(dirname + "/image_" + imageIndex.str() + ".png", frame->image);
        Mat depth;
        frame->depth.convertTo(depth, CV_16UC1, 1000);
        imwrite(dirname + "/depth_" + imageIndex.str() + ".png", depth);
        imwrite(dirname + "/mask_" + imageIndex.str() + ".png", frame->mask);
        {
            FileStorage fs(dirname + "/normals_" + imageIndex.str() + ".xml.gz", FileStorage::WRITE);
            CV_Assert(fs.isOpened());
            fs << "normals" << frame->normals;
        }

        imwrite(dirname + "/object_mask_" + imageIndex.str() + ".png", objectMasks[i]);
        {
            FileStorage fs(dirname + "/pose" + imageIndex.str() + ".xml.gz", FileStorage::WRITE);
            CV_Assert(fs.isOpened());
            fs << "pose" << poses[i];
        }
    }

    FileStorage fs(dirname + "/poseLinks.xml.gz", FileStorage::WRITE);
    CV_Assert(fs.isOpened());
    fs << "poseLinks" << "[";
    for(size_t i = 0; i < keyframePosesLinks.size(); i++)
    {
        const PosesLink& link = keyframePosesLinks[i];
        fs << "{";
        fs << "srcIndex" << link.srcIndex;
        fs << "dstIndex" << link.dstIndex;
        fs << "Rt" << link.Rt;
        fs << "}";
    }
    fs << "]";
}

void TrajectoryFrames::load(const std::string& dirname)
{
    cout << "TrajectoryFrames::load" << endl;

    clear();

    vector<string> frameIndices;
    readFrameIndices(dirname, frameIndices);
    if(frameIndices.empty())
    {
        cout << "Can not load the data from given directory of the base: " << dirname << endl;
        return;
    }

    frames.resize(frameIndices.size());
    objectMasks.resize(frameIndices.size());
    poses.resize(frameIndices.size());

#pragma omp parallel for
    for(size_t i = 0; i < frameIndices.size(); i++)
    {
        Mat image, depth;
        loadFrameData(dirname, frameIndices[i], image, depth);
        CV_Assert(!image.empty());
        CV_Assert(!depth.empty());

        Mat mask, objectMask;
        mask = imread(dirname + "/mask_" + frameIndices[i] + ".png", 0);
        objectMask = imread(dirname + "/object_mask_" + frameIndices[i] + ".png", 0);
        CV_Assert(!mask.empty());
        CV_Assert(!objectMask.empty());

        Mat normals;
        {
            FileStorage fs(dirname + "/normals_" + frameIndices[i] + ".xml.gz", FileStorage::READ);
            CV_Assert(fs.isOpened());
            fs["normals"] >> normals;
            CV_Assert(!normals.empty());
        }

        Mat pose;
        {
            FileStorage fs(dirname + "/pose" + frameIndices[i] + ".xml.gz", FileStorage::READ);
            CV_Assert(fs.isOpened());
            fs["pose"] >> pose;
            CV_Assert(!pose.empty());
        }

        Ptr<RgbdFrame> frame = new RgbdFrame(image, depth, mask, normals, atoi(frameIndices[i].c_str()));

        frames[i] = frame;
        objectMasks[i] = objectMask;
        poses[i] = pose;
    }

    resumeFrameState = TrajectoryFrames::KEYFRAME;
    frameStates.resize(frames.size(), TrajectoryFrames::KEYFRAME);

    FileStorage fs(dirname + "/poseLinks.xml.gz", FileStorage::READ);
    CV_Assert(fs.isOpened());
    FileNode fn = fs["poseLinks"];
    FileNodeIterator fnIt = fn.begin(), fnEnd = fn.end();
    for(; fnIt != fnEnd; ++fnIt)
    {
        int srcIndex = -1, dstIndex = -1;
        Mat Rt;
        (*fnIt)["srcIndex"] >> srcIndex;
        (*fnIt)["dstIndex"] >> dstIndex;
        (*fnIt)["Rt"] >> Rt;
        CV_Assert(srcIndex >= 0);
        CV_Assert(dstIndex >= 0);
        keyframePosesLinks.push_back(PosesLink(srcIndex, dstIndex, Rt));
    }
}

////////////////////////

CircularCaptureServer::FramePushOutput::FramePushOutput()
    : frameState(0)
{}

CircularCaptureServer::CircularCaptureServer() :
    maxCorrespColorDiff(DEFAULT_MAX_CORRESP_COLOR_DIFF),
    maxCorrespDepthDiff(DEFAULT_MAX_CORRESP_DEPTH_DIFF()),
    minInliersRatio(DEFAULT_MIN_INLIERS_RATIO()),
    skippedTranslation(DEFAULT_SKIPPED_TRANSLATION()),
    minTranslationDiff(DEFAULT_MIN_TRANSLATION_DIFF()),
    maxTranslationDiff(DEFAULT_MAX_TRANSLATION_DIFF()),
    minRotationDiff(DEFAULT_MIN_ROTATION_DIFF()),
    maxRotationDiff(DEFAULT_MAX_ROTATION_DIFF()),
    isInitialied(false),
    isFinalized(false)
{}

void CircularCaptureServer::filterImage(const Mat& src, Mat& dst) const
{
    dst = src; // TODO maybe median
    //medianBlur(src, dst, 3);
}

void CircularCaptureServer::firterDepth(const Mat& src, Mat& dst) const
{
    dst = src; // TODO maybe bilateral
}

Ptr<CircularCaptureServer::FramePushOutput> CircularCaptureServer::push(const Mat& _image, const Mat& _depth, int frameID)
{
    Ptr<FramePushOutput> pushOutput = new FramePushOutput();

    CV_Assert(isInitialied);
    CV_Assert(!isFinalized);

    CV_Assert(!normalsComputer.empty());
    CV_Assert(!tableMasker.empty());
    CV_Assert(!odometry.empty());

    if(isTrajectoryBroken)
    {
        cout << "frame " << frameID << ": trajectory was broken starting from keyframe " << (*trajectoryFrames->frames.rbegin())->ID << "." << endl;
        return pushOutput;
    }

    if(isLoopClosed)
    {
        cout << "frame " << frameID << ": loop is already closed" << endl;
        return pushOutput;
    }

    if(_image.empty() || _depth.empty())
    {
        cout << "Warning: Empty frame " << frameID << endl;
        return pushOutput;
    }

    //color information is ingored now but can be used in future
    Mat _grayImage = _image;
    if (_image.channels() == 3)
        cvtColor(_image, _grayImage, CV_BGR2GRAY);

    CV_Assert(_grayImage.type() == CV_8UC1);
    CV_Assert(_depth.type() == CV_32FC1);
    CV_Assert(_grayImage.size() == _depth.size());

    Mat image, depth;
    filterImage(_grayImage, image);
    filterImage(_depth, depth);

    Mat cloud;
    depthTo3d(depth, cameraMatrix, cloud);

    Mat normals;
    (*normalsComputer)(cloud, normals);

    Mat tableWithObjectMask;
    bool isTableMaskOk = (*tableMasker)(cloud, normals, tableWithObjectMask, &pushOutput->objectMask);
    pushOutput->frame = new OdometryFrame(_grayImage, _depth, tableWithObjectMask, normals, frameID);
    if(!isTableMaskOk)
    {
        cout << "Warning: bad table mask for the frame " << frameID << endl;
        return pushOutput;
    }

    //Ptr<OdometryFrameCache> currFrame = new OdometryFrameCache(image, depth, tableWithObjectMask);
    Ptr<OdometryFrame> currFrame = pushOutput->frame;

    if(lastKeyframe.empty())
    {
        firstKeyframe = currFrame;
        pushOutput->frameState |= TrajectoryFrames::KEYFRAME;
        pushOutput->pose = Mat::eye(4,4,CV_64FC1);
        cout << endl << "FIRST KEYFRAME ID " << frameID << endl;
    }
    else
    {
        // find frame to frame motion transformations
        {
            Mat Rt;
            cout << "odometry " << frameID << " -> " << prevFrameID << endl;
            if(odometry->compute(currFrame, prevFrame, Rt) && computeInliersRatio(currFrame, prevFrame, Rt, cameraMatrix, maxCorrespColorDiff,
                                                                                  maxCorrespDepthDiff) >= minInliersRatio)
            {
                pushOutput->frameState |= TrajectoryFrames::VALIDFRAME;
            }

            pushOutput->pose = prevPose * Rt;
            if((pushOutput->frameState & TrajectoryFrames::VALIDFRAME) != TrajectoryFrames::VALIDFRAME)
            {
                cout << "Warning: Bad odometry (too far motion or low inliers ratio) " << frameID << "->" << prevFrameID << endl;
                return pushOutput;
            }
        }

        // check for the current frame: is it keyframe?
        {
            int lastKeyframePoseIndex = -1;
            for(int i = trajectoryFrames->frames.size() - 1; i >= 0; i--)
            {
                if((trajectoryFrames->frameStates[i] & TrajectoryFrames::KEYFRAME) == TrajectoryFrames::KEYFRAME)
                {
                    lastKeyframePoseIndex = i;
                    break;
                }
            }
            CV_Assert(lastKeyframePoseIndex >= 0);
            Mat Rt = (trajectoryFrames->poses[lastKeyframePoseIndex]).inv(DECOMP_SVD) * pushOutput->pose;
            float tnorm = tvecNorm(Rt);
            float rnorm = rvecNormDegrees(Rt);

            if(tnorm > maxTranslationDiff || rnorm > maxRotationDiff)
            {
                cout << "Camera trajectory is broken (starting from " << (*trajectoryFrames->frames.rbegin())->ID << " frame)." << endl;
                cout << checkDataMessage << endl;
                isTrajectoryBroken = true;
                return pushOutput;
            }

            if((tnorm >= minTranslationDiff || rnorm >= minRotationDiff)) // we don't check inliers ratio here because it was done by frame-to-frame above
            {
                translationSum += tnorm;
                if(isLoopClosing)
                    cout << "possible ";
                cout << endl << "KEYFRAME ID " << frameID << endl;
                pushOutput->frameState |= TrajectoryFrames::KEYFRAME;
            }
        }

        // match with the first keyframe
        if(translationSum > skippedTranslation) // ready for closure
        {
            Mat Rt;
            if(odometry->compute(currFrame, firstKeyframe, Rt))
            {
                // we check inliers ratio for the loop closure frames because we didn't do this before
                float inliersRatio = computeInliersRatio(currFrame, firstKeyframe, Rt, cameraMatrix, maxCorrespColorDiff, maxCorrespDepthDiff);
                if(inliersRatio > minInliersRatio)
                {
                    if(inliersRatio >= closureInliersRatio)
                    {
                        isLoopClosing = true;
                        closureInliersRatio = inliersRatio;
                        closureFrame = currFrame;
                        closureFrameID = frameID;
                        closurePoseWithFirst = Rt;
                        closurePose = pushOutput->pose;
                        closureObjectMask = pushOutput->objectMask;
                        closureBgrImage = _image;
                        isClosureFrameKey = (pushOutput->frameState & TrajectoryFrames::KEYFRAME) == TrajectoryFrames::KEYFRAME;
                    }
                    else if(isLoopClosing)
                    {
                        isLoopClosed = true;
                    }
                }
                else if(isLoopClosing)
                {
                    isLoopClosed = true;
                }
            }
            else if(isLoopClosing)
            {
                isLoopClosed = true;
            }
        }
    }

    if((pushOutput->frameState & trajectoryFrames->resumeFrameState) == trajectoryFrames->resumeFrameState)
    {
        trajectoryFrames->push(new RgbdFrame(_image, _depth, currFrame->mask, currFrame->normals, frameID),
                               pushOutput->pose, pushOutput->objectMask, pushOutput->frameState);

        if((pushOutput->frameState & TrajectoryFrames::KEYFRAME) == TrajectoryFrames::KEYFRAME)
            lastKeyframe = currFrame;
    }

    prevFrame = currFrame;
    prevFrameID = frameID;
    prevPose = pushOutput->pose.clone();

    return pushOutput;
}

void CircularCaptureServer::reset()
{
    trajectoryFrames = new TrajectoryFrames();

    firstKeyframe.release();
    lastKeyframe.release();
    prevFrame.release();
    closureFrame.release();

    prevPose.release();
    prevFrameID = -1;

    isTrajectoryBroken = false;
    isLoopClosing = false;
    isLoopClosed = false;
    translationSum = 0.;
    closureInliersRatio = 0.;
    closureFrameID = -1;
    isClosureFrameKey = false;

    closureBgrImage.release();
    closureObjectMask.release();
    closurePose.release();
    closurePoseWithFirst.release();

    isFinalized = false;
}

void CircularCaptureServer::initialize(const Size& frameResolution, int storeFramesWithState)
{
    CV_Assert(storeFramesWithState == TrajectoryFrames::VALIDFRAME || storeFramesWithState == TrajectoryFrames::KEYFRAME);

    reset();

    trajectoryFrames->resumeFrameState = storeFramesWithState;

    CV_Assert(!cameraMatrix.empty());
    CV_Assert(cameraMatrix.type() == CV_64FC1);
    CV_Assert(cameraMatrix.size() == Size(3,3));

    normalsComputer = new RgbdNormals(frameResolution.height, frameResolution.width, CV_32FC1, cameraMatrix); // inner
    if(tableMasker.empty())
        tableMasker = new TableMasker();
    tableMasker->set("cameraMatrix", cameraMatrix);

    if(odometry.empty())
        odometry = new RgbdOdometry();
    odometry->set("cameraMatrix", cameraMatrix);

    isInitialied = true;
}

Ptr<TrajectoryFrames> CircularCaptureServer::finalize()
{
    CV_Assert(isInitialied);
    CV_Assert(!isFinalized);

    if(!closureFrame.empty())
    {
        cout << endl << "Closure frame index " << closureFrame->ID << endl;
        if(!isClosureFrameKey && ((trajectoryFrames->resumeFrameState & TrajectoryFrames::KEYFRAME) == TrajectoryFrames::KEYFRAME))
            trajectoryFrames->push(new RgbdFrame(closureBgrImage, closureFrame->depth, closureFrame->mask, closureFrame->normals, closureFrameID),
                                   closurePose, closureObjectMask, TrajectoryFrames::KEYFRAME);
        // Clear frames that were got after the closure keyframe
        int wasteFrameCount = 0;
        for(int i = static_cast<int>(trajectoryFrames->frames.size()) - 1; i >= 0; i--)
        {
            if(trajectoryFrames->frames[i]->ID > closureFrameID)
                wasteFrameCount++;
            else
            {
                if(trajectoryFrames->frames[i]->ID == closureFrameID)
                    trajectoryFrames->frameStates[i] = TrajectoryFrames::KEYFRAME;
                break;
            }
        }
        if(wasteFrameCount > 0)
        {
            int framesCount = trajectoryFrames->frames.size() - wasteFrameCount;
            trajectoryFrames->frames.resize(framesCount);
            trajectoryFrames->frameStates.resize(framesCount);
            trajectoryFrames->objectMasks.resize(framesCount);
            trajectoryFrames->poses.resize(framesCount);
        }

        cout << "Last frame ID " << closureFrameID << endl;
        cout << "All frames count = " << trajectoryFrames->frames.size() << endl;
        int keyframesCount = 0;
        int lastKeyframeID = -1;
        CV_Assert(trajectoryFrames->frames.size() == trajectoryFrames->frameStates.size());
        for(size_t i = 0; i < trajectoryFrames->frames.size(); i++)
        {
            if((trajectoryFrames->frameStates[i] & TrajectoryFrames::KEYFRAME) == TrajectoryFrames::KEYFRAME)
            {
                lastKeyframeID = trajectoryFrames->frames[i]->ID;
                keyframesCount++;
            }
        }
        cout << "Keyframes count = " << keyframesCount << endl;
        cout << "Last keyframe ID = " << lastKeyframeID << endl << endl;

        isLoopClosed = true;
    }
    else
    {

        cout << "The algorithm can not make loop closure on given data. " << endl;
        cout << checkDataMessage << endl;
    }

    // Fill camera poses links
    trajectoryFrames->keyframePosesLinks.clear();
    CV_Assert((trajectoryFrames->frameStates[0] & TrajectoryFrames::KEYFRAME) == TrajectoryFrames::KEYFRAME);
    int keyframeIndex = 0;
    for(size_t i = 1; i < trajectoryFrames->frames.size(); i++)
    {
        if((trajectoryFrames->frameStates[i] & TrajectoryFrames::KEYFRAME) == TrajectoryFrames::KEYFRAME)
        {
            trajectoryFrames->keyframePosesLinks.push_back(PosesLink(keyframeIndex, i));
            keyframeIndex = i;
        }
    }
    CV_Assert((trajectoryFrames->frameStates[trajectoryFrames->frameStates.size()-1] & TrajectoryFrames::KEYFRAME) == TrajectoryFrames::KEYFRAME);

    if(!closureFrame.empty())
        trajectoryFrames->keyframePosesLinks.push_back(PosesLink(0, trajectoryFrames->poses.size()-1, closurePoseWithFirst.inv(DECOMP_SVD)));

    isFinalized = true;

    return trajectoryFrames;
}
