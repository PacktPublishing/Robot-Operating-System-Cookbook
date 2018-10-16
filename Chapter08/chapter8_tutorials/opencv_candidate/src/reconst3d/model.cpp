#include <opencv2/rgbd/rgbd.hpp>

#include <boost/thread/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv_candidate_reconst3d/reconst3d.hpp>

using namespace cv;
using namespace std;

static
bool isPlyFile(const string& filename)
{
    const int extSz = 3;
    if(static_cast<int>(filename.size()) <= extSz)
        return false;

    string ext = filename.substr(filename.size() - extSz, extSz);
    return ext == "ply";
}

static
pcl::PointXYZRGB cvtColorPoint(const Point3f& srcPoint, const Vec3b& color)
{
    pcl::PointXYZRGB dstPoint;
    dstPoint.x = srcPoint.x;
    dstPoint.y = srcPoint.y;
    dstPoint.z = srcPoint.z;
    dstPoint.r = color[2];
    dstPoint.g = color[1];
    dstPoint.b = color[0];
    return dstPoint;
}

static
pcl::PointXYZRGBNormal cvtColorNormalPoint(const Point3f& srcPoint, const Vec3b& color, const Point3f& normal)
{
    pcl::PointXYZRGBNormal dstPoint;
    dstPoint.x = srcPoint.x;
    dstPoint.y = srcPoint.y;
    dstPoint.z = srcPoint.z;
    dstPoint.r = color[2];
    dstPoint.g = color[1];
    dstPoint.b = color[0];
    dstPoint.normal_x = normal.x;
    dstPoint.normal_y = normal.y;
    dstPoint.normal_z = normal.z;
    return dstPoint;
}

static
void splitCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>& mergedCloud,
                pcl::PointCloud<pcl::PointXYZRGB>& points,
                pcl::PointCloud<pcl::Normal>& normals)
{
    normals.clear();
    for(size_t i = 0; i < mergedCloud.points.size(); i++)
    {
        const pcl::PointXYZRGBNormal np = mergedCloud.points[i];
        pcl::PointXYZRGB p;
        p.x = np.x; p.y = np.y; p.z = np.z;
        p.r = np.r; p.g = np.g; p.b = np.b;
        pcl::Normal n;
        n.normal_x = np.normal_x;
        n.normal_y = np.normal_y;
        n.normal_z = np.normal_z;
        points.push_back(p);
        normals.push_back(n);
    }
}

template<class T>
void voxelFilter(pcl::PointCloud<T>& cloud, double gridSize)
{
    if(gridSize > 0.f)
    {
        typename pcl::PointCloud<T>::ConstPtr cloudPtr = boost::make_shared<const pcl::PointCloud<T> >(cloud);
        pcl::PointCloud<T> cloudDownsampled;
        pcl::VoxelGrid<T> voxelGridFilter;
        voxelGridFilter.setLeafSize(gridSize, gridSize, gridSize);
        voxelGridFilter.setDownsampleAllData(true);
        voxelGridFilter.setInputCloud(cloudPtr);
        voxelGridFilter.filter(cloudDownsampled);
        cloudDownsampled.swap(cloud);
    }
}

//--------------------------------------------------------------------------------------------------------------------

ObjectModel::ObjectModel()
{}

ObjectModel::ObjectModel(const vector<Ptr<RgbdFrame> >& frames, const vector<Mat>& poses,
                         const Mat& cameraMatrix, const vector<int>& frameIndices)
{
    create(frames, poses, cameraMatrix, frameIndices);
}

void ObjectModel::create(const vector<Ptr<RgbdFrame> >& frames, const vector<Mat>& poses,
                         const Mat& cameraMatrix, const vector<int>& frameIndices)
{
    CV_Assert(frames.size() == poses.size());

    clear();

    cout << "Convert frames to model" << endl;
    size_t usedFrameCount = frameIndices.empty() ? frames.size() : frameIndices.size();
    for(size_t i = 0; i < usedFrameCount; i++)
    {
        int  frameIndex = frameIndices.empty() ? i : frameIndices[i];
        CV_Assert(frameIndex < static_cast<int>(frames.size()) && frameIndex >= 0);

        const Ptr<RgbdFrame>& frame = frames[frameIndex];

        CV_Assert(!frame->image.empty());
        CV_Assert(frame->depth.size() == frame->image.size());
        CV_Assert(frame->mask.size() == frame->image.size());
        CV_Assert(frame->normals.empty() || frame->normals.size() == frame->image.size());

        const int maskElemCount = countNonZero(frame->mask);
        colors.reserve(colors.size() + maskElemCount);
        points3d.reserve(points3d.size() + maskElemCount);
        if(!frame->normals.empty())
            normals.reserve(normals.size() + maskElemCount);

        Mat cloud;
        depthTo3d(frame->depth, cameraMatrix, cloud);
        Mat transfPoints3d;
        perspectiveTransform(cloud.reshape(3,1), transfPoints3d, poses[frameIndex]);
        transfPoints3d = transfPoints3d.reshape(3, cloud.rows);

        Mat transfNormals;
        if(!frame->normals.empty())
        {
            const Mat R = poses[frameIndex](Rect(0,0,3,3));
            transform(frame->normals.reshape(3,1), transfNormals, R);
            transfNormals = transfNormals.reshape(3, frame->normals.rows);
        }

        for(int y = 0, pointIndex = 0; y < frame->mask.rows; y++)
        {
            const uchar* maskRow = frame->mask.ptr<uchar>(y);
            for(int x = 0; x < frame->mask.cols; x++, pointIndex++)
            {
                if(maskRow[x] && isValidDepth(frame->depth.at<float>(y,x)))
                {
                    colors.push_back(frame->image.at<Vec3b>(y,x));
                    points3d.push_back(transfPoints3d.at<Point3f>(y,x));
                    if(!frame->normals.empty())
                    {
                        Point3f n = transfNormals.at<Point3f>(y,x);
                        normals.push_back(n);
                    }
                }
            }
        }
        CV_Assert(colors.size() == points3d.size());
        CV_Assert(normals.empty() || normals.size() == points3d.size());

        cameraPoses.push_back(poses[frameIndex].clone());
    }
}

void ObjectModel::clear()
{
    colors.clear();
    points3d.clear();
    normals.clear();

    cameraPoses.clear();
}

void ObjectModel::read_ply(const string& filename)
{
    clear();

    ifstream file(filename.c_str());
    if(!file.is_open())
    {
        cerr << "File " << filename << " can not be opened." << endl;
        return;
    }

    if(!isPlyFile(filename))
    {
        cerr << "File " << filename << " has incorrect extension." << endl;
        return;
    }

    bool isElementSet = false;
    int propertyCount = 0;
    bool arePropertiesCounted = false;
    while(!file.eof())
    {
        const int sz = 1024;
        char line_c[sz];
        file.getline(line_c, sz);

        string line = line_c;
        if(line.find("ply") != string::npos)
            continue;
        if(line.find("format") != string::npos)
            continue;

        if(!isElementSet)
        {
            if(line.find("element") != string::npos)
                isElementSet = true;
        }
        else
        {
            if(!arePropertiesCounted)
            {
                if (line.find("property") != string::npos)
                    ++propertyCount;
                else
                    arePropertiesCounted = true;
            }
        }

        if(strcmp("end_header", line_c) == 0)
            break;
    }

    const int allPropertiesCount = 9;
    if(propertyCount != allPropertiesCount)
    {
        cerr << "Unsupported count of properties." << filename << endl;
        return;
    }

    while(!file.eof())
    {
        Point3f p3d;
        file >> p3d.x >> p3d.y;
        if(file.eof()) // to process the end of file
            break;
        file >> p3d.z;
        points3d.push_back(p3d);

        Vec3i color;
        file >> color[0] >> color[1] >> color[2];
        colors.push_back(color);

        Point3f n;
        file >> n.x >> n.y >> n.z;
        normals.push_back(n);
    }

    file.close();

    CV_Assert(points3d.size() == colors.size());
    CV_Assert(points3d.size() == normals.size());
}

void ObjectModel::write_ply(const string& filename) const
{
    CV_Assert(points3d.size() == colors.size());
    CV_Assert(points3d.size() == normals.size());

    ofstream file(filename.c_str());
    if(!file.is_open())
    {
        cerr << "File " << filename << " can not be opened." << endl;
        return;
    }

    if(!isPlyFile(filename))
    {
        cerr << "File " << filename << " has incorrect extension." << endl;
        return;
    }

    file << "ply" << endl;
    file << "format ascii 1.0" << endl;
    file << "element vertex " << points3d.size() << endl;
    file << "property float x" << endl;
    file << "property float y" << endl;
    file << "property float z" << endl;
    file << "property uchar blue" << endl;
    file << "property uchar green" << endl;
    file << "property uchar red" << endl;
    file << "property float nx" << endl;
    file << "property float ny" << endl;
    file << "property float nz" << endl;
    file << "end_header" << endl;

    for(size_t i = 0; i < points3d.size(); i++)
    {
        const Point3f& p3d = points3d[i];
        file << p3d.x << " " << p3d.y << " " <<  p3d.z << " ";

        const Vec3b& color = colors[i];
        file << static_cast<int>(color[0]) << " " << static_cast<int>(color[1]) << " " <<  static_cast<int>(color[2]) << " ";

        const Point3f& n = normals[i];
        file << n.x << " " << n.y << " " <<  n.z << endl;
    }

    file.close();
}

void ObjectModel::show(float gridSize, bool withCameraPoses, int normalLevel) const
{
    if(normalLevel > 0 && normals.empty())
        CV_Error(CV_StsBadArg, "Can not show normals beause they are empty\n");

    CV_Assert(points3d.size() == colors.size());

    pcl::PointCloud<pcl::PointXYZRGBNormal> globalMergedCloud;
    for(size_t i = 0; i < points3d.size(); i++)
    {
        CV_Assert(isValidDepth(points3d[i].x));
        CV_Assert(isValidDepth(points3d[i].y));
        CV_Assert(isValidDepth(points3d[i].z));

        globalMergedCloud.push_back(cvtColorNormalPoint(points3d[i], colors[i], normalLevel > 0 ? normals[i] : Point3f()));
    }

    voxelFilter(globalMergedCloud, gridSize);

    pcl::PointCloud<pcl::PointXYZRGB> globalPoints;
    pcl::PointCloud<pcl::Normal> globalNormals;
    splitCloud(globalMergedCloud, globalPoints, globalNormals);

    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr xyzrgbPtr = boost::make_shared<const pcl::PointCloud<pcl::PointXYZRGB> >(globalPoints);
    pcl::PointCloud<pcl::Normal>::ConstPtr normalsPtr  = boost::make_shared<const pcl::PointCloud<pcl::Normal> >(globalNormals);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor(0.5, 0.5, 1);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(xyzrgbPtr);
    viewer->addPointCloud<pcl::PointXYZRGB>(xyzrgbPtr, rgb, "result");
    if(normalLevel > 0)
        viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(xyzrgbPtr, normalsPtr, normalLevel, 0.003, "normals");

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0, "result");
    viewer->initCameraParameters();

    if(withCameraPoses)
    {
        // add poses to view:
        // each pose is a sphere,
        // a line from each sphere is a translation to the previous pose but truncated to 5 cm
        CV_Assert(!cameraPoses[0].empty());
        CV_Assert(cameraPoses[0].type() == CV_64FC1);

        for(size_t i = 0; i < cameraPoses.size(); i++)
        {
            CV_Assert(!cameraPoses[i].empty());
            CV_Assert(cameraPoses[i].type() == CV_64FC1);

            pcl::PointXYZ begArrow, endArrow;

            begArrow.x = cameraPoses[i].at<double>(0,3);
            begArrow.y = cameraPoses[i].at<double>(1,3);
            begArrow.z = cameraPoses[i].at<double>(2,3);

            double gb = static_cast<double>(i)/cameraPoses.size();

            stringstream sphereName;
            sphereName << "sphere_" << i;
            if(i == 0)
            {
                viewer->addSphere(begArrow, 0.021, 1., 0., 0., sphereName.str());
                continue;
            }
            else
                viewer->addSphere(begArrow, 0.007, 1., gb, gb, sphereName.str());

            Mat Rt = cameraPoses[i].inv(DECOMP_SVD) * cameraPoses[i-1];
            Mat v = cameraPoses[i](Rect(0,0,3,3)) * Rt(Rect(3,0,1,3));

            const double arrowLengh = 0.05; //m
            double vNorm = norm(v);
            if(vNorm > arrowLengh)
                v *= arrowLengh / vNorm;

            endArrow.x = begArrow.x + v.at<double>(0);
            endArrow.y = begArrow.y + v.at<double>(1);
            endArrow.z = begArrow.z + v.at<double>(2);

            stringstream lineName;
            lineName << "line_" << i;

            viewer->addLine(begArrow, endArrow, 1., gb, gb, lineName.str());
        }
    }

    while(!viewer->wasStopped())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}
