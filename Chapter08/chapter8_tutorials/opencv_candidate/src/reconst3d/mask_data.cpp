#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <opencv_candidate_reconst3d/reconst3d.hpp>
#include "ocv_pcl_convert.hpp"

using namespace std;
using namespace cv;

TableMasker::TableMasker() :
    zFilterMin(DEFAULT_Z_FILTER_MIN()),
    zFilterMax(DEFAULT_Z_FILTER_MAX()),
    minTablePart(DEFAULT_MIN_TABLE_PART()),
    minOverlapRatio(DEFAULT_MIN_OVERLAP_RATIO()),
    erodeIters(DEFAULT_ERODE_ITERS),
    minObjectPartArea(DEFAULT_MIN_OBJECT_PART_AREA)
{}

static
float calcAverageDepth(const Mat& cloud, const Mat& mask)
{
    float avgDepth = 0.f;
    int pointsNumber = 0;
    for(int y = 0; y < mask.rows; y++)
    {
        const uchar* maskRow = mask.ptr<uchar>(y);
        const Point3f* cloudRow = cloud.ptr<Point3f>(y);
        for(int x = 0; x < mask.cols; x++)
        {
            if(maskRow[x])
            {
                avgDepth += cloudRow[x].z;
                pointsNumber++;
            }
        }
    }
    CV_Assert(pointsNumber > 0);
    avgDepth /= pointsNumber;

    return avgDepth;
}

static void
splitPlaneMask(const Mat& planeMask, vector<Mat>& masks, float minMaskArea)
{
    masks.clear();

    vector<vector<Point> > contours;
    Mat planeMaskClone = planeMask.clone();
    findContours(planeMaskClone, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    if(contours.empty())
        return;

    for(size_t i = 0; i < contours.size(); i++)
    {
        int area = contourArea(contours[i]);
        if(area < minMaskArea)
            continue;

        Mat mask = Mat::zeros(planeMask.size(), CV_8UC1);
        drawContours(mask, contours, i, Scalar(255), CV_FILLED, 8);
        masks.push_back(mask & planeMask);
    }
}

static void
refineObjectMask(Mat& objectMask, int minObjectPartArea)
{
    vector<vector<Point> > contours;
    Mat objectMaskClone = objectMask.clone();
    findContours(objectMaskClone, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    if(contours.empty())
        return;

    for(size_t i = 0; i < contours.size(); i++)
    {
        int area = contourArea(contours[i]);
        if(area < minObjectPartArea)
            drawContours(objectMask, contours, i, Scalar(0), CV_FILLED, 8);
    }
}

int TableMasker::findTablePlane(const Mat& cloud, const Mat& prevMask,
                                const Mat_<uchar>& planesMask, size_t planesCount,
                                Mat& tableMask) const
{
    const float minTableArea = minTablePart * cloud.total();

    int planeIndex = -1;
    float tableAvgDepth = FLT_MAX;
    float maxOverlapRatio = 0.f;

    tableMask.release();
    for(size_t i = 0; i < planesCount; i++)
    {
        Mat curMask = planesMask == i;

        vector<Mat> splitedMasks;
        splitPlaneMask(curMask, splitedMasks, minTableArea);

        for(size_t splitIndex = 0; splitIndex < splitedMasks.size(); splitIndex++)
        {
            curMask = splitedMasks[splitIndex];

            float curOverlapRatio = 0.f;
            if(!prevMask.empty())
            {
                int intersectArea = countNonZero(prevMask & curMask);
                int unionArea = countNonZero(prevMask | curMask);
                if(unionArea > 0)
                    curOverlapRatio = static_cast<float>(intersectArea) / unionArea;

                if(curOverlapRatio < minOverlapRatio || curOverlapRatio < maxOverlapRatio)
                    continue;

                if(curOverlapRatio > maxOverlapRatio)
                {
                    planeIndex = i;
                    tableAvgDepth = calcAverageDepth(cloud, curMask);
                    maxOverlapRatio = curOverlapRatio;
                    tableMask = curMask;
                }
            }

            float curAvgDepth = calcAverageDepth(cloud, curMask);

            if(tableAvgDepth > curAvgDepth)
            {
                planeIndex = i;
                tableAvgDepth = curAvgDepth;
                maxOverlapRatio = curOverlapRatio;
                tableMask = curMask;
            }
        }
    }

    return planeIndex;
}

Mat TableMasker::calcObjectMask(const Mat& cloud,
                                const Mat& tableMask, const Vec4f& tableCoeffitients) const
{
    Mat erodedTableMask;
    erode(tableMask, erodedTableMask, Mat(), Point(-1,-1), erodeIters);

    // Find a mask of the object. For this find convex hull for the table and
    // get the points that are in the prism corresponding to the hull lying above the table.

    // Convert the data to pcl types
    pcl::PointCloud<pcl::PointXYZ> pclTableCloud;
    cvtCloud_cv2pcl(cloud, erodedTableMask, pclTableCloud);

    pcl::ModelCoefficients pclTableCoeffiteints;
    pclTableCoeffiteints.values.resize(4);
    pclTableCoeffiteints.values[0] = tableCoeffitients[0];
    pclTableCoeffiteints.values[1] = tableCoeffitients[1];
    pclTableCoeffiteints.values[2] = tableCoeffitients[2];
    pclTableCoeffiteints.values[3] = tableCoeffitients[3];

    // Find convex hull
    pcl::ConvexHull<pcl::PointXYZ> pclHullRreconstruntor;
    pcl::PointCloud<pcl::PointXYZ> pclTableHull;
    pclHullRreconstruntor.setInputCloud(boost::make_shared<const pcl::PointCloud<pcl::PointXYZ> >(pclTableCloud));
    pclHullRreconstruntor.setDimension(2);
    pclHullRreconstruntor.reconstruct(pclTableHull);

    // Get indices of points in the prism
    pcl::PointIndices pclPrismPointsIndices;
    pcl::ExtractPolygonalPrismData<pcl::PointXYZ> pclPrismSegmentator;
    pclPrismSegmentator.setHeightLimits(zFilterMin, zFilterMax);
    pcl::PointCloud<pcl::PointXYZ> pclCloud;
    cvtCloud_cv2pcl(cloud, Mat(), pclCloud);
    pclPrismSegmentator.setInputCloud(boost::make_shared<const pcl::PointCloud<pcl::PointXYZ> >(pclCloud));
    pclPrismSegmentator.setInputPlanarHull(boost::make_shared<const pcl::PointCloud<pcl::PointXYZ> >(pclTableHull));
    pclPrismSegmentator.segment(pclPrismPointsIndices);

    // Get points from the prism
    pcl::PointCloud<pcl::PointXYZ> pclPrismPoints;
    pcl::ExtractIndices<pcl::PointXYZ> extract_object_indices;
    extract_object_indices.setInputCloud(boost::make_shared<const pcl::PointCloud<pcl::PointXYZ> >(pclCloud));
    extract_object_indices.setIndices(boost::make_shared<const pcl::PointIndices>(pclPrismPointsIndices));
    extract_object_indices.filter(pclPrismPoints);

    // Draw the object points to the mask
    vector<Point3f> objectCloud;
    cvtCloud_pcl2cv(pclPrismPoints, objectCloud);

    Mat_<uchar> dirtyObjectMask = Mat::zeros(tableMask.size(), CV_8UC1);
    if(!objectCloud.empty())
    {
        vector<Point2f> objectPoints2d;
        projectPoints(objectCloud, Mat::zeros(3,1,CV_32FC1), Mat::zeros(3,1,CV_32FC1), cameraMatrix, Mat(), objectPoints2d);
        Rect r(0, 0, cloud.cols, cloud.rows);
        for(size_t i = 0; i < objectPoints2d.size(); i++)
        {
            if(r.contains(objectPoints2d[i]))
                dirtyObjectMask(objectPoints2d[i]) = 255;
        }
    }

    Mat objectMask = dirtyObjectMask & ~tableMask;
    refineObjectMask(objectMask, minObjectPartArea);

    return objectMask;
}


bool TableMasker::operator()(const Mat& cloud, const Mat& normals,
                             Mat& tableWithObjectMask, Mat* _objectMask, Vec4f* tableCoeffs) const
{
    Mat tableMask, objectMask;
    bool isOk = (*this)(cloud, normals, Mat(), tableMask, objectMask, tableCoeffs);

    if(isOk)
    {
        tableWithObjectMask = tableMask | objectMask;
        if(_objectMask)
            *_objectMask = objectMask.clone();
    }

    return isOk;
}


bool TableMasker::operator()(const Mat& cloud, const Mat& normals, const Mat& prevTableMask,
                             Mat& tableMask, Mat& objectMask, Vec4f* tableCoeffs) const
{
    CV_Assert(!cloud.empty() && cloud.type() == CV_32FC3);
    CV_Assert(!normals.empty() && normals.type() == CV_32FC3);
    CV_Assert(cloud.size() == normals.size());

    CV_Assert(!cameraMatrix.empty());

    const int minTableArea = static_cast<int>(minTablePart * cloud.total());

    if(!prevTableMask.empty())
    {
        CV_Assert(prevTableMask.size() == cloud.size());
        CV_Assert(prevTableMask.type() == CV_8UC1);
        CV_Assert(countNonZero(prevTableMask) >= minTableArea);
    }

    if(!planeComputer)
    {
        planeComputer = new RgbdPlane();
        planeComputer->set("sensor_error_a", 0.0075f);
    }
    planeComputer->set("min_size", minTableArea);

    Mat_<uchar> planesMask;
    vector<Vec4f> planesCoeffs;
    (*planeComputer)(cloud, normals, planesMask, planesCoeffs);

    int planeIndex = findTablePlane(cloud, prevTableMask, planesMask, planesCoeffs.size(), tableMask);
    if(planeIndex < 0)
        return false;

    objectMask = calcObjectMask(cloud, tableMask, planesCoeffs[planeIndex]);

    if(tableCoeffs)
        (*tableCoeffs) = planesCoeffs[planeIndex];

    return true;
}
