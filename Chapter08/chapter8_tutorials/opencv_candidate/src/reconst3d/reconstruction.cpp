#include <boost/thread/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

#include <opencv2/rgbd/rgbd.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "graph_optimizations.hpp"

using namespace cv;
using namespace std;

static void
preparePosesLinksWithoutRt(const vector<PosesLink>& srcLinks, vector<PosesLink>& dstLinks)
{
    dstLinks.resize(srcLinks.size());
    for(size_t i = 0; i < dstLinks.size(); i++)
        dstLinks[i] = PosesLink(srcLinks[i].srcIndex, srcLinks[i].dstIndex);
}

static void
preparePosesLinksWithoutRt(const vector<int>& poseIndices, vector<PosesLink>& dstLinks)
{
    CV_Assert(poseIndices.size() > 2);
    dstLinks.resize(poseIndices.size() - 1);
    for(size_t i = 1; i < poseIndices.size(); i++)
        dstLinks[i-1] = PosesLink(poseIndices[i-1], poseIndices[i]);
}

static void
prepareFramesForModelRefinement(const Ptr<TrajectoryFrames>& trajectoryFrames, const vector<int>& frameIndices, vector<Ptr<RgbdFrame> >& dstFrames)
{
    dstFrames.resize(trajectoryFrames->frames.size());
    for(size_t frameIndex = 0; frameIndex < dstFrames.size(); frameIndex++)
    {
        const Ptr<RgbdFrame> srcFrame = trajectoryFrames->frames[frameIndex];
        if(std::find(frameIndices.begin(), frameIndices.end(), frameIndex) != frameIndices.end())
        {
            // clone data for used frames because we can modify them
            dstFrames[frameIndex] = new RgbdFrame(srcFrame->image.clone(), srcFrame->depth.clone(),
                                                  trajectoryFrames->objectMasks[frameIndex].clone(), srcFrame->normals.clone(),
                                                  srcFrame->ID);
        }
        else
        {
            dstFrames[frameIndex] = new RgbdFrame(srcFrame->image, srcFrame->depth,
                                                  trajectoryFrames->objectMasks[frameIndex], srcFrame->normals,
                                                  srcFrame->ID);
        }
    }
}

static Mat
estimateTablePlane(const std::vector<cv::Ptr<cv::RgbdFrame> >& frames, const std::vector<cv::Mat>& objectMasks, const std::vector<cv::Mat>& poses,
                   const cv::Mat& cameraMatrix, const std::vector<int>& frameIndices=std::vector<int>())
{
    size_t usedFrameCount = frameIndices.empty() ? frames.size() : frameIndices.size();
    pcl::PointCloud<pcl::PointXYZ> tableCloud;
    for(size_t i = 0; i < usedFrameCount; i++)
    {
        int frameIndex = frameIndices.empty() ? i : frameIndices[i];
        CV_Assert(frameIndex < static_cast<int>(frames.size()) && frameIndex >= 0);

        const Ptr<RgbdFrame>& frame = frames[frameIndex];

        CV_Assert(!frame->depth.empty());
        CV_Assert(frame->depth.size() == frame->mask.size());

        const int maskElemCount = countNonZero(frame->mask);
        tableCloud.points.reserve(tableCloud.points.size() + maskElemCount);

        Mat cloud;
        depthTo3d(frame->depth, cameraMatrix, cloud);
        Mat transfPoints3d;
        perspectiveTransform(cloud.reshape(3,1), transfPoints3d, poses[frameIndex]);
        transfPoints3d = transfPoints3d.reshape(3, cloud.rows);

        Mat tableMask = frame->mask & ~objectMasks[frameIndex];
        for(int y = 0, pointIndex = 0; y < tableMask.rows; y++)
        {
            const uchar* maskRow = tableMask.ptr<uchar>(y);
            for(int x = 0; x < frame->mask.cols; x++, pointIndex++)
                if(maskRow[x] && isValidDepth(frame->depth.at<float>(y,x)))
                {
                    Point3f cv_p = transfPoints3d.at<Point3f>(y,x);
                    pcl::PointXYZ pcl_p;
                    pcl_p.x = cv_p.x; pcl_p.y = cv_p.y; pcl_p.z = cv_p.z;
                    tableCloud.points.push_back(pcl_p);
                }
        }
    }

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients (true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(boost::make_shared<const pcl::PointCloud<pcl::PointXYZ> >(tableCloud));

    seg.segment(*inliers, *coefficients);

    CV_Assert(!inliers->indices.empty());

    Mat tableCoefficients = Mat(coefficients->values).clone();
    CV_Assert(tableCoefficients.type() == CV_32FC1);

    // TODO: It's not absolutely correct, but it's how Rgbd.Normals selects a normal direction. Fix it.
    if(tableCoefficients.at<float>(2) > 0)
        tableCoefficients *= -1.f;

    return tableCoefficients.reshape(1,1);
}

ModelReconstructor::ModelReconstructor()
    : isShowStepResults(false),
      isEstimateRefinedTablePlane(false),
      maxBAPosesCount(DEFAULT_MAX_BA_POSES_COUNT)
{}

void ModelReconstructor::reconstruct(const Ptr<TrajectoryFrames>& trajectoryFrames, const Mat& cameraMatrix,
                                     Ptr<ObjectModel>& model) const
{
    CV_Assert(trajectoryFrames);
    CV_Assert(!trajectoryFrames->frames.empty());
    CV_Assert(trajectoryFrames->poses.size() == trajectoryFrames->frames.size());
    CV_Assert(!trajectoryFrames->keyframePosesLinks.empty());

    const float voxelSize = 0.005f;
    if(isShowStepResults)
    {
        cout << "Frame-to-frame odometry result" << endl;
        ObjectModel(trajectoryFrames->frames, trajectoryFrames->poses, cameraMatrix).show(voxelSize, true);
    }

    vector<Mat> refinedPosesSE3;
    vector<int> frameIndices;
    refineGraphSE3(trajectoryFrames->poses, trajectoryFrames->keyframePosesLinks, refinedPosesSE3, frameIndices);

    if(isShowStepResults)
    {
        cout << "Result of the loop closure" << endl;
        ObjectModel(trajectoryFrames->frames, refinedPosesSE3, cameraMatrix, frameIndices).show(voxelSize, true);
    }

    // fill posesLinks with empty Rt because we want that they will be recomputed
    vector<PosesLink> keyframePosesLinks;

    if(maxBAPosesCount > 0 && maxBAPosesCount < static_cast<int>(frameIndices.size())-1)
    {
        vector<int> subsetIndices;
        selectPosesSubset(refinedPosesSE3, frameIndices, subsetIndices, maxBAPosesCount);
        std::sort(subsetIndices.begin(), subsetIndices.end());
        preparePosesLinksWithoutRt(subsetIndices, keyframePosesLinks);
    }
    else
    {
        preparePosesLinksWithoutRt(trajectoryFrames->keyframePosesLinks, keyframePosesLinks);
    }

    vector<Mat> refinedPosesSE3RgbdICP;
    const float pointsPart = 0.05f;
    refineGraphSE3RgbdICP(trajectoryFrames->frames, refinedPosesSE3,
                          keyframePosesLinks, cameraMatrix, pointsPart, refinedPosesSE3RgbdICP, frameIndices);

    if(isShowStepResults)
    {
        cout << "Result of RgbdICP for camera poses" << endl;
        ObjectModel(trajectoryFrames->frames, refinedPosesSE3RgbdICP, cameraMatrix, frameIndices).show(0.001, true);
    }

    Mat tablePlane;
    if(isEstimateRefinedTablePlane)
        tablePlane = estimateTablePlane(trajectoryFrames->frames, trajectoryFrames->objectMasks, refinedPosesSE3RgbdICP, cameraMatrix, frameIndices);

    vector<Ptr<RgbdFrame> > objectFrames; // with mask for object points only,
                                          // they will modified while refining the object points
    prepareFramesForModelRefinement(trajectoryFrames, frameIndices, objectFrames);

    {
        vector<Mat> refinedPosesSE3RgbdICP2;
        refineGraphSE3RgbdICP(objectFrames, refinedPosesSE3RgbdICP,
                              keyframePosesLinks, cameraMatrix, 1, refinedPosesSE3RgbdICP2, frameIndices);

        refinedPosesSE3RgbdICP = refinedPosesSE3RgbdICP2;

        if(isShowStepResults)
            ObjectModel(objectFrames, refinedPosesSE3RgbdICP, cameraMatrix, frameIndices).show(0.001, true);
    }

    vector<Mat> refinedSE3ICPSE3ModelPoses;
    refineGraphSE3RgbdICPModel(objectFrames, refinedPosesSE3RgbdICP,
                               keyframePosesLinks, cameraMatrix, refinedSE3ICPSE3ModelPoses, frameIndices);

    model = new ObjectModel(objectFrames, refinedSE3ICPSE3ModelPoses, cameraMatrix, frameIndices);
    model->tablePlane = tablePlane;


    if(isShowStepResults)
    {
        cout << "Result of RgbdICP  for camera poses and model points refinement" << endl;
        model->show(0.001, true);
    }
}
