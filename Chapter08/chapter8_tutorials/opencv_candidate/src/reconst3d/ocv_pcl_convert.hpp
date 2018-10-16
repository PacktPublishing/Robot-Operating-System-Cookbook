#ifndef RECONST3D_OCV_PCL_CONVERT
#define RECONST3D_OCV_PCL_CONVERT

#include <opencv2/core/core.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

inline
void cvtCloud_cv2pcl(const cv::Mat& ocvCloud, const cv::Mat& mask, pcl::PointCloud<pcl::PointXYZ>& pclCloud)
{
    pclCloud.clear();
    for(int y = 0; y < ocvCloud.rows; y++)
    {
        const uchar* maskRow = mask.empty() ? 0 : mask.ptr<uchar>(y);
        const cv::Point3f* ocvCloudRow = ocvCloud.ptr<cv::Point3f>(y);
        for(int x = 0; x < ocvCloud.cols; x++)
        {
            if(!maskRow || maskRow[x])
            {
                pcl::PointXYZ pcl_p;
                const cv::Point3f& ocv_p = ocvCloudRow[x];
                pcl_p.x = ocv_p.x;
                pcl_p.y = ocv_p.y;
                pcl_p.z = ocv_p.z;
                pclCloud.push_back(pcl_p);
            }
        }
    }
}

inline
void cvtCloud_pcl2cv(const pcl::PointCloud<pcl::PointXYZ>& pclCloud, std::vector<cv::Point3f>& ocvCloud)
{
    ocvCloud.resize(pclCloud.size());
    for(size_t i = 0; i < pclCloud.size(); i++)
    {
        const pcl::PointXYZ& src = pclCloud.points[i];
        cv::Point3f& dst = ocvCloud[i];
        dst.x = src.x;
        dst.y = src.y;
        dst.z = src.z;
    }
}

#endif
