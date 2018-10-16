//#include <cstdio>
#include <iostream>

#include <opencv_candidate_reconst3d/reconst3d.hpp>

#include <opencv2/core/core.hpp>
#include <opencv_candidate/feature2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/rgbd/rgbd.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>

using namespace std;
using namespace cv;

static
void filterMatchesWithInvalidDepth(const vector<KeyPoint>& srcKeypoints, const vector<KeyPoint>& dstKeypoints,
                                   const Mat& srcCloud, const Mat& dstCloud,
                                   vector<DMatch>& matches)
{
    CV_Assert(srcCloud.type() == CV_32FC3);
    CV_Assert(dstCloud.type() == CV_32FC3);

    vector<DMatch> filteredMatches;
    filteredMatches.reserve(matches.size());
    for(size_t i = 0; i < matches.size(); i++)
    {
        const DMatch& m = matches[i];
        if(isValidDepth(srcCloud.at<Point3f>(srcKeypoints[m.queryIdx].pt).z) &&
           isValidDepth(dstCloud.at<Point3f>(dstKeypoints[m.trainIdx].pt).z))
        {
            filteredMatches.push_back(m);
        }
    }

    swap(matches, filteredMatches);
}

static
void generateRandomIndices(vector<int>& indices, const Range& range)
{
    RNG& rng = theRNG();
    for(size_t i = 0; i < indices.size(); i++)
    {
        bool unique = false;
        do
        {
            indices[i] = rng.uniform(range.start, range.end);
            unique = find(indices.begin(), indices.begin() + i, indices[i]) == (indices.begin() + i);
        }
        while(!unique);
    }
}

static inline
bool is3dConsistentMatches(const Mat& srcCloud, const Mat& dstCloud,
                           const vector<KeyPoint>& srcKeypoints, const vector<KeyPoint>& dstKeypoints,
                           const vector<DMatch>& matches,
                           const vector<int>& matchIndices, float maxDistDiff3d)
{
    // TODO check for angles (samples don't have to belong one line)
    // TODO optimaze for the case of 3 points?
    const float minDist = 0.1f; // TODO move this parameter to the interface
    for(size_t i = 0; i < matchIndices.size(); i++)
    {
        const DMatch& m0 = matches[matchIndices[i]];
        const Point3f& srcPoint0 = srcCloud.at<Point3f>(srcKeypoints[m0.queryIdx].pt);
        const Point3f& dstPoint0 = dstCloud.at<Point3f>(dstKeypoints[m0.trainIdx].pt);

        for(size_t j = i+1; j < matchIndices.size(); j++)
        {
            const DMatch& m1 = matches[matchIndices[j]];
            const Point3f& srcPoint1 = srcCloud.at<Point3f>(srcKeypoints[m1.queryIdx].pt);
            const Point3f& dstPoint1 = dstCloud.at<Point3f>(dstKeypoints[m1.trainIdx].pt);

            float srcDist = cv::norm(srcPoint0 - srcPoint1);
            float dstDist = cv::norm(dstPoint0 - dstPoint1);

            if(srcDist < minDist || dstDist < minDist )
                return false;

            if(abs(srcDist - dstDist) > maxDistDiff3d)
                return false;
        }
    }
    return true;
}

/* This is an implementation of the algorithm from the paper:
    K.S. Arun, T.S. Huang, S.D. Blostein “Least-Squares Fitting of Two 3-D Point Sets”,
*/
static
Mat computeTransformation(const Mat& srcCloud, const Mat& dstCloud,
                          const vector<KeyPoint>& srcKeypoints, const vector<KeyPoint>& dstKeypoints, const vector<DMatch>& matches,
                          const vector<int>& matchIndices)
{
    // compute points centers
    Mat srcPoints3d(matchIndices.size(), 1, CV_32FC3),
        dstPoints3d(matchIndices.size(), 1, CV_32FC3);
    for(size_t i = 0; i < matchIndices.size(); i++)
    {
        const DMatch& m = matches[matchIndices[i]];
        srcPoints3d.at<Point3f>(i) = srcCloud.at<Point3f>(srcKeypoints[m.queryIdx].pt);
        dstPoints3d.at<Point3f>(i) = dstCloud.at<Point3f>(dstKeypoints[m.trainIdx].pt);
    }
    srcPoints3d.convertTo(srcPoints3d, CV_64FC3);
    dstPoints3d.convertTo(dstPoints3d, CV_64FC3);
    srcPoints3d = srcPoints3d.reshape(1,srcPoints3d.rows);
    dstPoints3d = dstPoints3d.reshape(1,dstPoints3d.rows);

    Mat meanSrcPoint, meanDstPoint;
    reduce(srcPoints3d, meanSrcPoint, 0, CV_REDUCE_AVG);
    reduce(dstPoints3d, meanDstPoint, 0, CV_REDUCE_AVG);

    // Comupte H
    Mat H = Mat::zeros(3,3,CV_64FC1);
    for(size_t i = 0; i < matchIndices.size(); i++)
        H += (srcPoints3d.row(i) - meanSrcPoint).t() * (dstPoints3d.row(i) - meanDstPoint);

    SVD svd(H);
    Mat v = svd.vt.t();
    Mat R = v * svd.u.t();
    if(determinant(R) < 0.)
    {
        v.col(2) = -1 * v.col(2);
        R = v * svd.u.t();
    }
    Mat t = meanDstPoint.t() - R * meanSrcPoint.t();

    Mat Rt = Mat::eye(4,4,CV_64FC1);
    R.copyTo(Rt(Rect(0,0,3,3)));
    t.copyTo(Rt(Rect(3,0,1,3)));

    return Rt;
}

static inline
Point2f projectPoint(const Point3f& p3d, const Mat& Rt, double fx, double fy, double cx, double cy)
{
    CV_Assert(Rt.type() == CV_64FC1);

    const double * Rt_ptr = Rt.ptr<const double>();
    Point2f p2d;

    double pz = Rt_ptr[8] * p3d.x + Rt_ptr[9] * p3d.y + Rt_ptr[10] * p3d.z + Rt_ptr[11];
    double pz_inv = 1./pz;
    p2d.x = pz_inv * fx * (Rt_ptr[0] * p3d.x + Rt_ptr[1] * p3d.y + Rt_ptr[2] * p3d.z + Rt_ptr[3]) + cx;
    p2d.y = pz_inv * fy * (Rt_ptr[4] * p3d.x + Rt_ptr[5] * p3d.y + Rt_ptr[6] * p3d.z + Rt_ptr[7]) + cy;

    return p2d;
}

static
void computeInliers(const Mat& srcCloud, const Mat& K, const Mat& Rt,
                    const vector<KeyPoint>& srcKeypoints, const vector<KeyPoint>& dstKeypoints,
                    const vector<DMatch>& matches,
                    vector<DMatch>& inliers, float maxPointsDist2d)
{
    CV_Assert(K.type() == CV_32FC1);
    const double fx = K.at<float>(0,0);
    const double fy = K.at<float>(1,1);
    const double cx = K.at<float>(0,2);
    const double cy = K.at<float>(1,2);
    for(size_t i = 0; i < matches.size(); i++)
    {
        const DMatch& m = matches[i];
        const Point3f& srcPoint3d = srcCloud.at<Point3f>(srcKeypoints[m.queryIdx].pt);
        Point2f transfSrcPoint2d = projectPoint(srcPoint3d, Rt, fx, fy, cx, cy);
        if(norm(transfSrcPoint2d - dstKeypoints[m.trainIdx].pt) <= maxPointsDist2d)
            inliers.push_back(m);
    }
}

///////////////////////////////////////////////////////////////////////////////////////

Feature2dPoseEstimator::Feature2dPoseEstimator() :
    minInliersCount(DEFAULT_MIN_INLIERS_COUNT),
    reliableInliersCount(DEFAULT_RELIABLE_INLIERS_COUNT),
    ransacMaxIterCount(DEFAULT_RANSAC_MAX_ITER_COUNT),
    maxPointsDist2d(DEFAULT_MAX_POINTS_DIST2D()),
    maxDistDiff3d(DEFAULT_MAX_DIST_DIFF3D())
{}

Mat Feature2dPoseEstimator::operator()(const vector<KeyPoint>& srcKeypoints, const Mat& srcDescriptors, const Mat& srcCloud,
                                       const vector<KeyPoint>& dstKeypoints, const Mat& dstDescriptors, const Mat& dstCloud,
                                       vector<DMatch>* _matches) const
{
    CV_Assert(!cameraMatrix.empty());
    CV_Assert(static_cast<int>(srcKeypoints.size()) == srcDescriptors.rows);
    CV_Assert(static_cast<int>(dstKeypoints.size()) == dstDescriptors.rows);
    CV_Assert(!srcCloud.empty());
    CV_Assert(srcCloud.size() == dstCloud.size());

    vector<DMatch> matches;
    BFMatcher matcher(NORM_L2SQR, true);
    matcher.match(srcDescriptors, dstDescriptors, matches);

    filterMatchesWithInvalidDepth(srcKeypoints, dstKeypoints, srcCloud, dstCloud, matches);

    Mat Rt = estimateRt(srcKeypoints, dstKeypoints, srcCloud, dstCloud, matches);

    if(_matches)
        matches.swap(*_matches);

    return Rt;
}

Mat Feature2dPoseEstimator::estimateRt(const vector<KeyPoint>& srcKeypoints, const vector<KeyPoint>& dstKeypoints,
                                       const Mat& srcCloud, const Mat& dstCloud,
                                       vector<DMatch>& matches) const
{
    Mat resRt;
    if(static_cast<int>(matches.size()) < minInliersCount)
        return Mat();

    const int ransacSampleCount = 3;
    vector<int> matchIndices(ransacSampleCount);
    Range matchIndicesRange(0, matches.size());

    // TODO threading

    vector<DMatch> resInliers;
    for(int iter = 0; iter < ransacMaxIterCount; iter++)
    {
        generateRandomIndices(matchIndices, matchIndicesRange);

        bool is3dConsistent = is3dConsistentMatches(srcCloud, dstCloud, srcKeypoints, dstKeypoints, matches, matchIndices, maxDistDiff3d);

        if(!is3dConsistent)
            continue;

        Mat Rt = computeTransformation(srcCloud, dstCloud, srcKeypoints, dstKeypoints, matches, matchIndices);
        if(Rt.empty())
            continue;

        vector<DMatch> inliers;
        computeInliers(srcCloud, cameraMatrix, Rt,
                       srcKeypoints, dstKeypoints, matches, inliers, maxPointsDist2d);

        if(static_cast<int>(inliers.size()) > reliableInliersCount)
        {
            resRt = Rt;
            swap(resInliers, inliers);
            break;
        }

        if(inliers.size() > resInliers.size())
        {
            resRt = Rt;
            swap(resInliers, inliers);
        }
    }

    if(static_cast<int>(resInliers.size()) <= minInliersCount)
        return Mat();

    swap(resInliers, matches);

//    vector<int> resMatchIndices(resInliers.size());
//    for(size_t i = 0; i < resMatchIndices.size(); i++)
//        resMatchIndices[i] = i;
//    resRt = computeTransformation(srcCloud, dstCloud, srcKeypoints, dstKeypoints, resInliers, resMatchIndices);

    return resRt;
}
