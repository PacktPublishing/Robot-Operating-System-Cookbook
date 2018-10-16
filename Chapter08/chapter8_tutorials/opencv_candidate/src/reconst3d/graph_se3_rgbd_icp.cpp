#include <g2o/core/factory.h>
#include <g2o/types/icp/types_icp.h>

#include <opencv2/imgproc/imgproc.hpp>

#include "ocv_pcl_convert.hpp"
#include "graph_optimizations.hpp"

using namespace std;
using namespace cv;

const double sobelScale = 1./8.;
const double grayNormalScale = 1./255.;

static
void downsamplePoints(const Mat& points, vector<int>& downsampledIndices, size_t count)
{
    CV_Assert(count >= 2);
    CV_Assert(points.total() >= count);
    CV_Assert(points.type() == CV_32FC3);
    CV_Assert(points.cols == 1 || points.rows == 1);

    downsampledIndices.resize(count);

    //TODO: optimize by exploiting symmetry in the distance matrix
    Mat dists = Mat::zeros(points.total(), points.total(), CV_32FC1);
    for(int i = 0; i < dists.rows; i++)
    {
        for(int j = i; j < dists.cols; j++)
        {
            float dist = (float)norm(points.at<Point3f>(i) - points.at<Point3f>(j));
            dists.at<float>(j,i) = dists.at<float>(i,j) = dist;
        }
    }

    double maxVal;
    Point maxLoc;
    minMaxLoc(dists, 0, &maxVal, 0, &maxLoc);

    downsampledIndices[0] = maxLoc.x;
    downsampledIndices[1] = maxLoc.y;

    Mat activedDists(0, dists.cols, dists.type());
    activedDists.push_back(dists.row(maxLoc.y));

    Mat candidatePointsMask(1, dists.cols, CV_8UC1, Scalar(255));
    candidatePointsMask.at<uchar>(0, maxLoc.y) = 0;

    for(size_t i = 2; i < count; i++)
    {
        activedDists.push_back(dists.row(maxLoc.x));
        candidatePointsMask.at<uchar>(0, maxLoc.x) = 0;

        Mat minDists;
        reduce(activedDists, minDists, 0, CV_REDUCE_MIN);
        minMaxLoc(minDists, 0, &maxVal, 0, &maxLoc, candidatePointsMask);
        downsampledIndices[i] = maxLoc.x;
    }
}

void selectPosesSubset(const vector<Mat>& poses,
                       const vector<int>& indices,
                       vector<int>& selectedIndices, size_t count)
{
    if(indices.size() <= count)
    {
        selectedIndices = indices;
        return;
    }

    vector<Point3f> origins(indices.size());
    vector<int> totalIndices(indices.size());
    for(size_t i = 0; i < indices.size(); i++)
    {
        const Mat_<double>& pose = poses[indices[i]];

        Point3f origin;
        origin.x = pose(0,3);
        origin.y = pose(1,3);
        origin.z = pose(2,3);

        origins[i] = origin;
        totalIndices[i] = indices[i];
    }

    downsamplePoints(Mat(origins), selectedIndices, count);

    for(size_t i = 0; i < selectedIndices.size(); i++)
        selectedIndices[i] = totalIndices[selectedIndices[i]];
}

static inline
Eigen::Matrix<double, 2, 3> create_dPdG(const Mat& K, const Eigen::Vector3d& G)
{
    Eigen::Matrix<double, 2, 3> dPdG;
    double z_inv = 1. / G[2];

    dPdG(0,0) = K.at<double>(0,0) * z_inv;
    dPdG(0,1) = 0.;
    dPdG(0,2) = -dPdG(0,0) * G[0] * z_inv;

    dPdG(1,0) = 0.;
    dPdG(1,1) = K.at<double>(1,1) * z_inv;
    dPdG(1,2) = -dPdG(1,1) * G[1] * z_inv;

    return dPdG;
}

namespace g2o {

    class EdgeRGBD
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
        uchar srcColor, dstColor;
        short int dst_dIdx, dst_dIdy;
        Vector3d srcP3d;
        Mat const* K;

        EdgeRGBD() : srcColor(0), dstColor(0), dst_dIdx(0), dst_dIdy(0), K(0)
        {}
    };

    class Edge_V_V_RGBD : public  BaseBinaryEdge<1, EdgeRGBD, VertexSE3, VertexSE3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Edge_V_V_RGBD() {}
        Edge_V_V_RGBD(const Edge_V_V_RGBD* e)
          : BaseBinaryEdge<1, EdgeRGBD, VertexSE3, VertexSE3>()
        {
            _vertices[0] = const_cast<HyperGraph::Vertex*> (e->vertex(0));
            _vertices[1] = const_cast<HyperGraph::Vertex*> (e->vertex(1));

            _measurement.srcColor = e->measurement().srcColor;
            _measurement.dstColor = e->measurement().dstColor;

            _measurement.dst_dIdx = e->measurement().dst_dIdx;
            _measurement.dst_dIdy = e->measurement().dst_dIdy;

            _measurement.srcP3d = e->measurement().srcP3d;

            _measurement.K = e->measurement().K;
        }

      // I/O functions
      virtual bool read(std::istream& /*is*/) {return false;}
      virtual bool write(std::ostream& /*os*/) const {return false;}

      // return the error estimate as a 3-vector
      void computeError()
      {
          double w = grayNormalScale;
          double err = w * ((static_cast<double>(measurement().dstColor) -
                             static_cast<double>(measurement().srcColor)));

          _error.data()[0] = err;
      }

      virtual void linearizeOplus()
      {
          const VertexSE3 *src_vp = static_cast<const VertexSE3*>(_vertices[0]);
          const VertexSE3 *dst_vp = static_cast<const VertexSE3*>(_vertices[1]);

          Vector3d dstP3d = dst_vp->estimate().inverse() * src_vp->estimate() * measurement().srcP3d;

          double w = grayNormalScale * sobelScale;

          const EdgeRGBD& meas = measurement();
          Matrix<double, 1, 2> dst_dIdP;
          dst_dIdP(0,0) = w * static_cast<double>(meas.dst_dIdx);
          dst_dIdP(0,1) = w * static_cast<double>(meas.dst_dIdy);

          Matrix<double, 2, 3> dPdG = create_dPdG(*(meas.K), dstP3d);
          if (!src_vp->fixed())
          {
              Matrix3d Rsrc2dst = dst_vp->estimate().matrix().topLeftCorner<3,3>().transpose() *
                                    src_vp->estimate().matrix().topLeftCorner<3,3>();
              Matrix<double, 3, 6> dGdT;
              dGdT.block<3,3>(0,0) = Rsrc2dst;
              dGdT.block<3,1>(0,3) = Rsrc2dst * dRidx.transpose() * meas.srcP3d;
              dGdT.block<3,1>(0,4) = Rsrc2dst * dRidy.transpose() * meas.srcP3d;
              dGdT.block<3,1>(0,5) = Rsrc2dst * dRidz.transpose() * meas.srcP3d;
              _jacobianOplusXi = dst_dIdP * dPdG * dGdT;
          }

          if (!dst_vp->fixed())
          {
              Matrix<double, 3, 6> dGdT;
              dGdT.block<3,3>(0,0) = -Matrix3d::Identity();
              dGdT.block<3,1>(0,3) = dRidx * dstP3d;
              dGdT.block<3,1>(0,4) = dRidy * dstP3d;
              dGdT.block<3,1>(0,5) = dRidz * dstP3d;
              _jacobianOplusXj = dst_dIdP * dPdG * dGdT;
          }
        }
      static Matrix3d dRidx;
      static Matrix3d dRidy;
      static Matrix3d dRidz; // differential quat matrices


      static void initializeStaticMatrices()
      {
          //if(dRidx.data() == 0)
          {
              dRidx << 0.0,0.0,0.0,
                0.0,0.0,2.0,
                0.0,-2.0,0.0;
              dRidy  << 0.0,0.0,-2.0,
                0.0,0.0,0.0,
                2.0,0.0,0.0;
              dRidz  << 0.0,2.0,0.0,
                -2.0,0.0,0.0,
                0.0,0.0,0.0;
          }
      }
    };

    Matrix3d Edge_V_V_RGBD::dRidx; // differential quat matrices
    Matrix3d Edge_V_V_RGBD::dRidy; // differential quat matrices
    Matrix3d Edge_V_V_RGBD::dRidz; // differential quat matrices


    G2O_REGISTER_TYPE(Edge_V_V_RGBD, Edge_V_V_RGBD);
}

//---------------------------------------------------------------------------------------------------------
// this function is from *Odometry implementation
static
int computeCorresps(const Mat& K, const Mat& K_inv, const Mat& Rt,
                    const Mat& depth0, const Mat& validMask0,
                    const Mat& depth1, const Mat& selectMask1, float maxDepthDiff,
                    Mat& corresps)
{
    CV_Assert(K.type() == CV_64FC1);
    CV_Assert(K_inv.type() == CV_64FC1);
    CV_Assert(Rt.type() == CV_64FC1);

    corresps.create(depth1.size(), CV_32SC1);
    corresps.setTo(-1);

    Rect r(0, 0, depth1.cols, depth1.rows);
    Mat Kt = Rt(Rect(3,0,1,3)).clone();
    Kt = K * Kt;
    const double * Kt_ptr = reinterpret_cast<const double *>(Kt.ptr());

    AutoBuffer<float> buf(3 * (depth1.cols + depth1.rows));
    float *KRK_inv0_u1 = buf;
    float *KRK_inv1_v1_plus_KRK_inv2 = KRK_inv0_u1 + depth1.cols;
    float *KRK_inv3_u1 = KRK_inv1_v1_plus_KRK_inv2 + depth1.rows;
    float *KRK_inv4_v1_plus_KRK_inv5 = KRK_inv3_u1 + depth1.cols;
    float *KRK_inv6_u1 = KRK_inv4_v1_plus_KRK_inv5 + depth1.rows;
    float *KRK_inv7_v1_plus_KRK_inv8 = KRK_inv6_u1 + depth1.cols;
    {
        Mat R = Rt(Rect(0,0,3,3)).clone();

        Mat KRK_inv = K * R * K_inv;
        const double * KRK_inv_ptr = reinterpret_cast<const double *>(KRK_inv.ptr());
        for(int u1 = 0; u1 < depth1.cols; u1++)
        {
            KRK_inv0_u1[u1] = KRK_inv_ptr[0] * u1;
            KRK_inv3_u1[u1] = KRK_inv_ptr[3] * u1;
            KRK_inv6_u1[u1] = KRK_inv_ptr[6] * u1;
        }

        for(int v1 = 0; v1 < depth1.rows; v1++)
        {
            KRK_inv1_v1_plus_KRK_inv2[v1] = KRK_inv_ptr[1] * v1 + KRK_inv_ptr[2];
            KRK_inv4_v1_plus_KRK_inv5[v1] = KRK_inv_ptr[4] * v1 + KRK_inv_ptr[5];
            KRK_inv7_v1_plus_KRK_inv8[v1] = KRK_inv_ptr[7] * v1 + KRK_inv_ptr[8];
        }
    }

    int correspCount = 0;
    for(int v1 = 0; v1 < depth1.rows; v1++)
    {
        const float *depth1_row = depth1.ptr<float>(v1);
        const uchar *mask1_row = selectMask1.ptr<uchar>(v1);
        for(int u1 = 0; u1 < depth1.cols; u1++)
        {
            float d1 = depth1_row[u1];
            if(mask1_row[u1])
            {
                CV_DbgAssert(!cvIsNaN(d1));
                float transformed_d1 = static_cast<float>(d1 * (KRK_inv6_u1[u1] + KRK_inv7_v1_plus_KRK_inv8[v1]) + Kt_ptr[2]);
                if(transformed_d1 > 0)
                {
                    float transformed_d1_inv = 1.f / transformed_d1;
                    int u0 = cvRound(transformed_d1_inv * (d1 * (KRK_inv0_u1[u1] + KRK_inv1_v1_plus_KRK_inv2[v1]) + Kt_ptr[0]));
                    int v0 = cvRound(transformed_d1_inv * (d1 * (KRK_inv3_u1[u1] + KRK_inv4_v1_plus_KRK_inv5[v1]) + Kt_ptr[1]));

                    if(r.contains(Point(u0,v0)))
                    {
                        float d0 = depth0.at<float>(v0,u0);
                        if(validMask0.at<uchar>(v0, u0) && std::abs(transformed_d1 - d0) <= maxDepthDiff)
                        {
                            CV_DbgAssert(!cvIsNaN(d0));
                            int c = corresps.at<int>(v0,u0);
                            if(c != -1)
                            {
                                int exist_u1, exist_v1;
                                get2shorts(c, exist_u1, exist_v1);

                                float exist_d1 = (float)(depth1.at<float>(exist_v1,exist_u1) *
                                    (KRK_inv6_u1[exist_u1] + KRK_inv7_v1_plus_KRK_inv8[exist_v1]) + Kt_ptr[2]);

                                if(transformed_d1 > exist_d1)
                                    continue;
                            }
                            else
                                correspCount++;

                            set2shorts(corresps.at<int>(v0,u0), u1, v1);
                        }
                    }
                }
            }
        }
    }
    return correspCount;
}

int computeCorrespsFiltered(const Mat& K, const Mat& K_inv, const Mat& Rt,
                            const Mat& depth0, const Mat& validMask0,
                            const Mat& depth1, const Mat& selectMask1, float maxDepthDiff,
                            Mat& corresps,
                            const Mat& normals0, const Mat& normals1,
                            const Mat& image0, const Mat& image1,
                            float maxColorDiff)
{
    const double maxNormalsDiff = 30; // in degrees
    const double maxNormalAngleDev = 75; // in degrees

    const double cosMaxNormalsDiff = std::cos(maxNormalsDiff / 180 * CV_PI);
    const double cosMaxNormalAngleDev = std::cos(maxNormalAngleDev / 180 * CV_PI);

    computeCorresps(K, K_inv, Rt, depth0, validMask0, depth1, selectMask1,
                    maxDepthDiff, corresps);

    int count = 0;
    const Point3f Oz_inv(0,0,-1); // TODO replace by vector to camera position?
    for(int v0 = 0; v0 < corresps.rows; v0++)
    {
        for(int u0 = 0; u0 < corresps.cols; u0++)
        {
            int c = corresps.at<int>(v0, u0);
            if(c != -1)
            {
                Point3f n0 = normals0.at<Point3f>(v0,u0);
                if(n0.ddot(Oz_inv) < cosMaxNormalAngleDev)
                {
                    corresps.at<int>(v0, u0) = -1;
                    continue;
                }

                int u1, v1;
                get2shorts(c, u1, v1);

                const Point3f& n1 = normals1.at<Point3f>(v1,u1);
                Point3f tn1 = rotatePoint(n1, Rt);

                if(n0.ddot(tn1) < cosMaxNormalsDiff)
                {
                    corresps.at<int>(v0, u0) = -1;
                    continue;
                }

                if(std::abs(image0.at<uchar>(v0,u0) - image1.at<uchar>(v1,u1)) > maxColorDiff)
                {
                    corresps.at<int>(v0, u0) = -1;
                    continue;
                }
                count++;
            }
        }
    }

    return count;
}

void fillGraphSE3RgbdICP(g2o::SparseOptimizer* optimizer, int pyramidLevel, const std::vector<Ptr<OdometryFrame> >& frames,
                         const std::vector<Mat>& poses, const std::vector<PosesLink>& posesLinks, const Mat& cameraMatrix_64F,
                         std::vector<int>& frameIndices,
                         double maxTranslation, double maxRotation, double maxDepthDiff)
{
    CV_Assert(frames.size() == poses.size());
    g2o::Edge_V_V_RGBD::initializeStaticMatrices(); // TODO: make this more correctly

    fillGraphSE3(optimizer, poses, posesLinks, frameIndices);

    // set up ICP edges
    for(size_t currIdx = 0; currIdx < frameIndices.size(); currIdx++)
    {
        int currFrameIdx = frameIndices[currIdx];

        const Mat& curCloud = frames[currFrameIdx]->pyramidCloud[pyramidLevel];
        const Mat& curNormals = frames[currFrameIdx]->pyramidNormals[pyramidLevel];

        for(size_t prevIdx = 0; prevIdx < frameIndices.size(); prevIdx++)
        {
            int prevFrameIdx = frameIndices[prevIdx];
            if(currFrameIdx == prevFrameIdx)
                continue;

            const Mat& prevCloud = frames[prevFrameIdx]->pyramidCloud[pyramidLevel];
            const Mat& prevNormals = frames[prevFrameIdx]->pyramidNormals[pyramidLevel];

            Mat curToPrevRt = poses[prevFrameIdx].inv(DECOMP_SVD) * poses[currFrameIdx];
            if(tvecNorm(curToPrevRt) > maxTranslation || rvecNormDegrees(curToPrevRt) > maxRotation)
                continue;

            Mat corresps_icp;
            CV_Assert(!frames[currFrameIdx]->pyramidMask.empty());
            CV_Assert(!frames[prevFrameIdx]->pyramidNormalsMask.empty());
            CV_Assert(!frames[prevFrameIdx]->pyramidTexturedMask.empty());
            int correspsCount_icp = computeCorrespsFiltered(cameraMatrix_64F, cameraMatrix_64F.inv(), curToPrevRt.inv(DECOMP_SVD),
                                                            frames[currFrameIdx]->pyramidDepth[pyramidLevel],
                                                            frames[currFrameIdx]->pyramidMask[pyramidLevel],
                                                            frames[prevFrameIdx]->pyramidDepth[pyramidLevel],
                                                            frames[prevFrameIdx]->pyramidNormalsMask[pyramidLevel],
                                                            maxDepthDiff, corresps_icp,
                                                            frames[currFrameIdx]->pyramidNormals[pyramidLevel],
                                                            frames[prevFrameIdx]->pyramidNormals[pyramidLevel],
                                                            frames[currFrameIdx]->pyramidImage[pyramidLevel],
                                                            frames[prevFrameIdx]->pyramidImage[pyramidLevel]);

            const int minCorrespsCount = 100;

            if(correspsCount_icp < minCorrespsCount)
                continue;
#define WITH_RGBD 1
#if WITH_RGBD
            const double rgbdScale = 1./(255 * std::max(cameraMatrix_64F.at<double>(0,0), cameraMatrix_64F.at<double>(1,1)));

            Mat corresps_rgbd;
            int correspsCount_rgbd = computeCorrespsFiltered(cameraMatrix_64F, cameraMatrix_64F.inv(), curToPrevRt.inv(DECOMP_SVD),
                                                             frames[currFrameIdx]->pyramidDepth[pyramidLevel],
                                                             frames[currFrameIdx]->pyramidMask[pyramidLevel],
                                                             frames[prevFrameIdx]->pyramidDepth[pyramidLevel],
                                                             frames[prevFrameIdx]->pyramidTexturedMask[pyramidLevel],
                                                             maxDepthDiff, corresps_rgbd,
                                                             frames[currFrameIdx]->pyramidNormals[pyramidLevel],
                                                             frames[prevFrameIdx]->pyramidNormals[pyramidLevel],
                                                             frames[currFrameIdx]->pyramidImage[pyramidLevel],
                                                             frames[prevFrameIdx]->pyramidImage[pyramidLevel]);
            if(correspsCount_rgbd < minCorrespsCount)
                continue;
#endif

            cout << currFrameIdx << " -> " << prevFrameIdx << ": icp correspondences count " << correspsCount_icp << endl;

#if WITH_RGBD
            cout << currFrameIdx << " -> " << prevFrameIdx << ": rgbd correspondences count " << correspsCount_rgbd << endl;
#endif

            // edges for poses
            for(int v0 = 0; v0 < corresps_icp.rows; v0++)
            {
                for(int u0 = 0; u0 < corresps_icp.cols; u0++)
                {
                    int c = corresps_icp.at<int>(v0, u0);
                    if(c == -1)
                        continue;

                    int u1, v1;
                    get2shorts(c, u1, v1);

                    {
                        g2o::Edge_V_V_GICP * e = new g2o::Edge_V_V_GICP();
                        e->setVertex(0, optimizer->vertex(prevFrameIdx));
                        e->setVertex(1, optimizer->vertex(currFrameIdx));

                        g2o::EdgeGICP meas;
                        meas.pos0 = cvtPoint_ocv2egn(prevCloud.at<Point3f>(v1,u1));
                        meas.pos1 = cvtPoint_ocv2egn(curCloud.at<Point3f>(v0,u0));
                        meas.normal0 = cvtPoint_ocv2egn(prevNormals.at<Point3f>(v1,u1));
                        meas.normal1 = cvtPoint_ocv2egn(curNormals.at<Point3f>(v0,u0));

                        e->setMeasurement(meas);
                        meas = e->measurement();
                        e->information() = meas.prec0(0.01);

                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);

                        optimizer->addEdge(e);

                    }
                }
            }
#if WITH_RGBD
            for(int v0 = 0; v0 < corresps_rgbd.rows; v0++)
            {
                for(int u0 = 0; u0 < corresps_rgbd.cols; u0++)
                {
                    int c = corresps_rgbd.at<int>(v0, u0);
                    if(c == -1)
                        continue;

                    int u1, v1;
                    get2shorts(c, u1, v1);

                    {
                        int srcIdx = currFrameIdx;
                        int dstIdx = prevFrameIdx;
                        g2o::Edge_V_V_RGBD * e = new g2o::Edge_V_V_RGBD();
                        e->setVertex(0, optimizer->vertex(srcIdx));
                        e->setVertex(1, optimizer->vertex(dstIdx));

                        g2o::EdgeRGBD meas;
                        meas.srcColor = frames[srcIdx]->pyramidImage[pyramidLevel].at<uchar>(v0, u0);
                        meas.dstColor = frames[dstIdx]->pyramidImage[pyramidLevel].at<uchar>(v1, u1);

                        meas.dst_dIdx = frames[dstIdx]->pyramid_dI_dx[pyramidLevel].at<short int>(v1, u1);
                        meas.dst_dIdy = frames[dstIdx]->pyramid_dI_dy[pyramidLevel].at<short int>(v1, u1);

                        meas.srcP3d = cvtPoint_ocv2egn(frames[srcIdx]->pyramidCloud[pyramidLevel].at<Point3f>(v0, u0));

                        meas.K = &cameraMatrix_64F;

                        e->setMeasurement(meas);
                        meas = e->measurement();
                        double w = rgbdScale * static_cast<double>(correspsCount_icp) / correspsCount_rgbd;
                        e->information() = Eigen::Matrix<double,1,1>::Identity() * w;

                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);

                        optimizer->addEdge(e);
                    }
                }
            }
#endif
        }
    }
}

static
void buildPyramidCameraMatrix(const Mat& cameraMatrix, int levels, vector<Mat>& pyramidCameraMatrix)
{
    pyramidCameraMatrix.resize(levels);

    Mat cameraMatrix_dbl;
    cameraMatrix.convertTo(cameraMatrix_dbl, CV_64FC1);

    for(int i = 0; i < levels; i++)
    {
        Mat levelCameraMatrix = i == 0 ? cameraMatrix_dbl : 0.5f * pyramidCameraMatrix[i-1];
        levelCameraMatrix.at<double>(2,2) = 1.;
        pyramidCameraMatrix[i] = levelCameraMatrix;
    }
}

void refineGraphSE3RgbdICP(const std::vector<Ptr<RgbdFrame> >& _frames,
                           const std::vector<Mat>& poses, const std::vector<PosesLink>& posesLinks,
                           const Mat& cameraMatrix, float pointsPart,
                           std::vector<Mat>& refinedPoses, std::vector<int>& frameIndices)
{
    CV_Assert(_frames.size() == poses.size());

    // TODO: find corresp to main API?
    const int levelsCount = 3;
    vector<float> minGradientMagnitudes(levelsCount);
    minGradientMagnitudes[0] = 10;
    minGradientMagnitudes[1] = 5;
    minGradientMagnitudes[2] = 1;
    vector<int> iterCounts(levelsCount);
    iterCounts[0] = 3;
    iterCounts[1] = 4;
    iterCounts[2] = 4;

    RgbdICPOdometry odom;
    odom.set("maxPointsPart", pointsPart);
    odom.set("minGradientMagnitudes", Mat(minGradientMagnitudes));
    odom.set("iterCounts", Mat(iterCounts));
    odom.set("cameraMatrix", cameraMatrix);

    std::vector<Ptr<OdometryFrame> > frames(_frames.size());
    for(size_t i = 0; i < frames.size(); i++)
    {
        Mat gray;
        CV_Assert(_frames[i]->image.channels() == 3);
        cvtColor(_frames[i]->image, gray, CV_BGR2GRAY);
        Ptr<OdometryFrame> frame = new OdometryFrame(gray, _frames[i]->depth, _frames[i]->mask,
                                                     _frames[i]->normals, _frames[i]->ID);
        odom.prepareFrameCache(frame, OdometryFrame::CACHE_ALL);
        frames[i] = frame;
    }

    refinedPoses.resize(poses.size());
    for(size_t i = 0; i < poses.size(); i++)
        refinedPoses[i] = poses[i].clone();

    vector<Mat> pyramidCameraMatrix;
    buildPyramidCameraMatrix(cameraMatrix, iterCounts.size(), pyramidCameraMatrix);

    for(int level = iterCounts.size() - 1; level >= 0; level--)
    {
        for(int iter = 0; iter < iterCounts[level]; iter++)
        {
            G2OLinearSolver* linearSolver =  createLinearSolver(DEFAULT_LINEAR_SOLVER_TYPE);
            G2OBlockSolver* blockSolver = createBlockSolver(linearSolver);
            g2o::OptimizationAlgorithm* nonLinerSolver = createNonLinearSolver(DEFAULT_NON_LINEAR_SOLVER_TYPE, blockSolver);
            g2o::SparseOptimizer* optimizer = createOptimizer(nonLinerSolver);

            double maxTranslation = DBL_MAX;
            double maxRotation = DBL_MAX;
            double maxDepthDiff = 0.07;
            if(level == 0)
            {
                maxTranslation = 0.20;
                maxRotation = 30;
            }

            fillGraphSE3RgbdICP(optimizer, level, frames, refinedPoses, posesLinks, pyramidCameraMatrix[level], frameIndices,
                                maxTranslation, maxRotation, maxDepthDiff);

            optimizer->initializeOptimization();
            const int optIterCount = 1;
            cout << "Vertices count: " << optimizer->vertices().size() << endl;
            cout << "Edges count: " << optimizer->edges().size() << endl;
            cout << "Start optimization " << endl;
            if(optimizer->optimize(optIterCount) != optIterCount)
            {
                optimizer->clear();
                delete optimizer;
                break;
            }
            cout << "Finish optimization " << endl;

            getSE3Poses(optimizer, frameIndices, refinedPoses);

            optimizer->clear();
            delete optimizer;
        }
    }
}
