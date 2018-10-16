#include <g2o/core/factory.h>
#include <g2o/types/icp/types_icp.h>

#include <opencv2/imgproc/imgproc.hpp>

#include "ocv_pcl_convert.hpp"
#include "graph_optimizations.hpp"

using namespace std;
using namespace cv;

namespace g2o {

    class Edge_V_V_GICPLandmark : public  BaseBinaryEdge<3, EdgeGICP, VertexPointXYZ, VertexSE3>
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Edge_V_V_GICPLandmark() {}
        Edge_V_V_GICPLandmark(const Edge_V_V_GICPLandmark* e);

      // I/O functions
      virtual bool read(std::istream& /*is*/) {return false;}
      virtual bool write(std::ostream& /*os*/) const {return false;}

      // return the error estimate as a 3-vector
      void computeError()
      {
        // from <ViewPoint> to <Point>
        const VertexPointXYZ *vp0 = static_cast<const VertexPointXYZ*>(_vertices[0]);
        const VertexSE3 *vp1 = static_cast<const VertexSE3*>(_vertices[1]);

        Vector3d p1 = vp1->estimate() * measurement().pos1;
        _error = p1 - vp0->estimate();
      }

      virtual void linearizeOplus();

      static Matrix3d dRidx;
      static Matrix3d dRidy;
      static Matrix3d dRidz;

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

    G2O_REGISTER_TYPE(Edge_V_V_GICPLandmark, Edge_V_V_GICPLandmark);

    Matrix3d Edge_V_V_GICPLandmark::dRidx; // differential quat matrices
    Matrix3d Edge_V_V_GICPLandmark::dRidy; // differential quat matrices
    Matrix3d Edge_V_V_GICPLandmark::dRidz; // differential quat matrices


    // Copy constructor
    Edge_V_V_GICPLandmark::Edge_V_V_GICPLandmark(const Edge_V_V_GICPLandmark* e)
      : BaseBinaryEdge<3, EdgeGICP, VertexPointXYZ, VertexSE3>()
    {

      // Temporary hack - TODO, sort out const-ness properly
      _vertices[0] = const_cast<HyperGraph::Vertex*> (e->vertex(0));
      _vertices[1] = const_cast<HyperGraph::Vertex*> (e->vertex(1));

      _measurement.pos0 = e->measurement().pos0;
      _measurement.pos1 = e->measurement().pos1;
      _measurement.normal0 = e->measurement().normal0;
      _measurement.normal1 = e->measurement().normal1;
      _measurement.R0 = e->measurement().R0;
      _measurement.R1 = e->measurement().R1;
    }

    void Edge_V_V_GICPLandmark::linearizeOplus()
    {
      //  std::cout << "START Edge_V_V_GICPLandmark::linearizeOplus() " << std::endl;
      VertexPointXYZ* vp0 = static_cast<VertexPointXYZ*>(_vertices[0]);
      VertexSE3* vp1 = static_cast<VertexSE3*>(_vertices[1]);
      Vector3d p1 = measurement().pos1;

      if (!vp0->fixed())
        {
          _jacobianOplusXi.block<3,3>(0,0) = -Matrix3d::Identity();
        }

      if (!vp1->fixed())
        {
          Matrix3d R1 = vp1->estimate().matrix().topLeftCorner<3,3>();
          _jacobianOplusXj.block<3,3>(0,0) = R1;
          _jacobianOplusXj.block<3,1>(0,3) = R1*dRidx.transpose()*p1;
          _jacobianOplusXj.block<3,1>(0,4) = R1*dRidy.transpose()*p1;
          _jacobianOplusXj.block<3,1>(0,5) = R1*dRidz.transpose()*p1;
        }
    }
}

void refineGraphSE3RgbdICPModel(std::vector<Ptr<RgbdFrame> >& _frames,
                                const std::vector<Mat>& poses, const std::vector<PosesLink>& posesLinks, const Mat& cameraMatrix,
                                std::vector<Mat>& refinedPoses, std::vector<int>& frameIndices)
{
    CV_Assert(_frames.size() == poses.size());
    g2o::Edge_V_V_GICPLandmark::initializeStaticMatrices(); // TODO: make this more correctly

    Mat cameraMatrix_64F, cameraMatrix_inv_64F;
    cameraMatrix.convertTo(cameraMatrix_64F, CV_64FC1);
    cameraMatrix_inv_64F = cameraMatrix_64F.inv();

    refinedPoses.resize(poses.size());
    for(size_t i = 0; i < poses.size(); i++)
        refinedPoses[i] = poses[i];

    RgbdICPOdometry odom;
    vector<float> minGradientMagnitudes(1,10);
    vector<int> iterCounts(1,10);
    odom.set("maxPointsPart", 1.);
    odom.set("minGradientMagnitudes", Mat(minGradientMagnitudes).clone());
    odom.set("iterCounts", Mat(iterCounts).clone());
    odom.set("cameraMatrix", cameraMatrix);

    std::vector<Ptr<OdometryFrame> > frames(_frames.size());
    for(size_t i = 0; i < frames.size(); i++)
    {
        //frames[i]->releasePyramids();
        Mat gray;
        CV_Assert(_frames[i]->image.channels() == 3);
        cvtColor(_frames[i]->image, gray, CV_BGR2GRAY);
        Ptr<OdometryFrame> fr = new OdometryFrame(gray, _frames[i]->depth, _frames[i]->mask,
                                                  _frames[i]->normals, _frames[i]->ID);
        odom.prepareFrameCache(fr, OdometryFrame::CACHE_ALL);
        frames[i] = fr;
    }

    const int iterCount = 3;//7
    const int minCorrespCount = 3;
    const float maxColorDiff = 50;

#if 1 // the RAM is enough
    const double maxTranslation = DBL_MAX;
    const double maxRotation = DBL_MAX;
#else
    // this version is less accurate because does not set up all corresps for each point
    // TODO optimize the memory usage
    const double maxTranslation = 0.20;
    const double maxRotation = 30;
#endif
    const double maxDepthDiff = 0.005;

    for(int iter = 0; iter < iterCount; iter++)
    {
        G2OLinearSolver* linearSolver =  createLinearSolver(DEFAULT_LINEAR_SOLVER_TYPE);
        G2OBlockSolver* blockSolver = createBlockSolver(linearSolver);
        g2o::OptimizationAlgorithm* nonLinerSolver = createNonLinearSolver(DEFAULT_NON_LINEAR_SOLVER_TYPE, blockSolver);
        g2o::SparseOptimizer* optimizer = createOptimizer(nonLinerSolver);

        fillGraphSE3RgbdICP(optimizer, 0, frames, refinedPoses, posesLinks, cameraMatrix_64F, frameIndices,
                            maxTranslation, maxRotation, maxDepthDiff);

        vector<Mat> vertexIndices(frameIndices.size());
        int vertexIdx = frames.size();
        for(size_t currIdx = 0; currIdx < frameIndices.size(); currIdx++)
        {
            int currFrameIdx = frameIndices[currIdx];

            vertexIndices[currIdx] = Mat(frames[currFrameIdx]->image.size(), CV_32SC1, Scalar(-1));

            Mat& curVertexIndices = vertexIndices[currIdx];
            const Mat& curCloud = frames[currFrameIdx]->pyramidCloud[0];
            const Mat& curNormals = frames[currFrameIdx]->normals;

            // compute count of correspondences
            Mat correspsCounts = Mat(frames[currFrameIdx]->image.size(), CV_32SC1, Scalar(0));
            for(size_t prevIdx = 0; prevIdx < frameIndices.size(); prevIdx++)
            {
                int prevFrameIdx = frameIndices[prevIdx];
                if(currFrameIdx == prevFrameIdx)
                    continue;

                Mat curToPrevRt = refinedPoses[prevFrameIdx].inv(DECOMP_SVD) * refinedPoses[currFrameIdx];
                if(tvecNorm(curToPrevRt) > maxTranslation || rvecNormDegrees(curToPrevRt) > maxRotation)
                    continue;

                Mat corresps;
                computeCorrespsFiltered(cameraMatrix_64F, cameraMatrix_inv_64F, curToPrevRt.inv(DECOMP_SVD),
                                        frames[currFrameIdx]->depth,
                                        frames[currFrameIdx]->pyramidMask[0],
                                        frames[prevFrameIdx]->depth,
                                        frames[prevFrameIdx]->pyramidNormalsMask[0],
                                        maxDepthDiff, corresps,
                                        frames[currFrameIdx]->pyramidNormals[0],
                                        frames[prevFrameIdx]->pyramidNormals[0],
                                        frames[currFrameIdx]->image,
                                        frames[prevFrameIdx]->image,
                                        maxColorDiff);

                for(int v0 = 0; v0 < corresps.rows; v0++)
                {
                    for(int u0 = 0; u0 < corresps.cols; u0++)
                    {
                        int c = corresps.at<int>(v0, u0);
                        if(c != -1)
                            correspsCounts.at<int>(v0,u0)++;
                    }
                }
            }

            // set up edges
            for(size_t prevIdx = 0; prevIdx < frameIndices.size(); prevIdx++)
            {
                int prevFrameIdx = frameIndices[prevIdx];
                if(currFrameIdx == prevFrameIdx)
                    continue;

                const Mat& prevCloud = frames[prevFrameIdx]->pyramidCloud[0];
                const Mat& prevNormals = frames[prevFrameIdx]->normals;
                Mat curToPrevRt = refinedPoses[prevFrameIdx].inv(DECOMP_SVD) * refinedPoses[currFrameIdx];
                if(tvecNorm(curToPrevRt) > maxTranslation || rvecNormDegrees(curToPrevRt) > maxRotation)
                    continue;

                Mat corresps;
                int correspsCount = computeCorrespsFiltered(cameraMatrix_64F, cameraMatrix_inv_64F, curToPrevRt.inv(DECOMP_SVD),
                                        frames[currFrameIdx]->depth,
                                        frames[currFrameIdx]->pyramidMask[0],
                                        frames[prevFrameIdx]->depth,
                                        frames[prevFrameIdx]->pyramidNormalsMask[0],
                                        maxDepthDiff, corresps,
                                        frames[currFrameIdx]->pyramidNormals[0],
                                        frames[prevFrameIdx]->pyramidNormals[0],
                                        frames[currFrameIdx]->image,
                                        frames[prevFrameIdx]->image,
                                        maxColorDiff);

                std::cout << "landmarks: iter " << iter << "; cur " << currFrameIdx << "; prev " << prevFrameIdx << "; corresps " << correspsCount << std::endl;

                // poses and edges for points3d
                for(int v0 = 0; v0 < corresps.rows; v0++)
                {
                    for(int u0 = 0; u0 < corresps.cols; u0++)
                    {
                        int c = corresps.at<int>(v0, u0);
                        if(c == -1)
                            continue;

                        if(correspsCounts.at<int>(v0,u0) < minCorrespCount)
                            continue;

                        int u1, v1;
                        get2shorts(c, u1, v1);

                        const Rect rect(0,0,curCloud.cols, curCloud.rows);
                        CV_Assert(rect.contains(Point(u1, v1)) && !cvIsNaN(prevCloud.at<Point3f>(v1,u1).x));
                        Eigen::Vector3d pt_prev, pt_cur, norm_prev, norm_cur, global_norm_prev;
                        {
                            pt_prev = cvtPoint_ocv2egn(prevCloud.at<Point3f>(v1,u1));
                            norm_prev = cvtPoint_ocv2egn(prevNormals.at<Point3f>(v1,u1));

                            global_norm_prev = cvtPoint_ocv2egn(rotatePoint(prevNormals.at<Point3f>(v1,u1), refinedPoses[prevFrameIdx]));
                            pt_cur = cvtPoint_ocv2egn(transformPoint(curCloud.at<Point3f>(v0,u0), refinedPoses[currFrameIdx]));
                            norm_cur = cvtPoint_ocv2egn(rotatePoint(curNormals.at<Point3f>(v0,u0), refinedPoses[currFrameIdx]));
                        }

                        // add new pose
                        if(curVertexIndices.at<int>(v0,u0) == -1)
                        {
                            g2o::VertexPointXYZ* modelPoint = new g2o::VertexPointXYZ;
                            modelPoint->setId(vertexIdx);
                            modelPoint->setEstimate(pt_cur);
                            modelPoint->setMarginalized(true);
                            optimizer->addVertex(modelPoint);

                            curVertexIndices.at<int>(v0,u0) = vertexIdx;
                            vertexIdx++;
                        }

                        int vidx = curVertexIndices.at<int>(v0,u0);

                        g2o::Edge_V_V_GICPLandmark * e = new g2o::Edge_V_V_GICPLandmark();
                        e->setVertex(0, optimizer->vertex(vidx));
                        e->setVertex(1, optimizer->vertex(prevFrameIdx));

                        g2o::EdgeGICP meas;
                        meas.pos0 = pt_cur;
                        meas.pos1 = pt_prev;
                        meas.normal0 = norm_cur;
                        meas.normal1 = norm_prev;

                        e->setMeasurement(meas);
                        meas = e->measurement();

//                                    e->information() = meas.prec0(0.01);
                        meas.normal1 = global_norm_prev; // to get global covariation
                        e->information() = 0.001 * (meas.cov0(1.).inverse() + meas.cov1(1.).inverse());
                        meas.normal1 = norm_prev; // set local normal

                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);

                        optimizer->addEdge(e);
                    }
                }
            }
        }

        optimizer->initializeOptimization();
        const int optIterCount = 1;
        cout << "Vertices count: " << optimizer->vertices().size() << endl;
        cout << "Edges count: " << optimizer->edges().size() << endl;
        if(optimizer->optimize(optIterCount) != optIterCount)
        {
            optimizer->clear();
            delete optimizer;
            break;
        }

        getSE3Poses(optimizer, frameIndices, refinedPoses);

        // update points poses
        cout << "Updating model points..." << endl;
        CV_Assert(frameIndices.size() ==  vertexIndices.size());
        for(size_t i = 0; i < frameIndices.size(); i++)
        {
            int frameIdx = frameIndices[i];
            const Mat& curVertexIndices = vertexIndices[i];
            Mat& depth = frames[frameIdx]->depth;
            for(int y = 0; y < curVertexIndices.rows; y++)
            {
                for(int x = 0; x < curVertexIndices.cols; x++)
                {
                    int vidx = curVertexIndices.at<int>(y,x);
                    if(vidx < 0)
                        continue;

                    Point3f p;
                    {
                        g2o::VertexPointXYZ* v = dynamic_cast<g2o::VertexPointXYZ*>(optimizer->vertex(vidx));
                        Eigen::Vector3d ep = v->estimate();
                        p.x = ep[0]; p.y = ep[1]; p.z = ep[2];
                    }
                    // TODO use new (x,y) using point projection?
                    depth.at<float>(y,x) = transformPoint(p, refinedPoses[frameIdx].inv(DECOMP_SVD)).z;
                }
            }

            frames[frameIdx]->pyramidMask.clear();
            frames[frameIdx]->pyramidDepth.clear();
            frames[frameIdx]->normals.release();
            frames[frameIdx]->pyramidNormals.clear();
            frames[frameIdx]->pyramidCloud.clear();
            frames[frameIdx]->pyramidNormalsMask.clear();
            odom.prepareFrameCache(frames[frameIdx], OdometryFrame::CACHE_ALL);
        }

        optimizer->clear();
        delete optimizer;
    }

    // remove points without correspondences
    for(size_t currIdx = 0; currIdx < frameIndices.size(); currIdx++)
    {
        int currFrameIdx = frameIndices[currIdx];
        // compute count of correspondences
        Mat& curCloud = frames[currFrameIdx]->pyramidCloud[0];
        Mat& curDepth = frames[currFrameIdx]->depth;
        Mat correspsCounts = Mat(frames[currFrameIdx]->image.size(), CV_32SC1, Scalar(0));
        for(size_t prevIdx = 0; prevIdx < frameIndices.size(); prevIdx++)
        {
            int prevFrameIdx = frameIndices[prevIdx];
            if(currFrameIdx == prevFrameIdx)
                continue;

            Mat curToPrevRt = refinedPoses[prevFrameIdx].inv(DECOMP_SVD) * refinedPoses[currFrameIdx];
            Mat corresps;
            computeCorrespsFiltered(cameraMatrix_64F, cameraMatrix_inv_64F, curToPrevRt.inv(DECOMP_SVD),
                                    frames[currFrameIdx]->depth,
                                    frames[currFrameIdx]->pyramidMask[0],
                                    frames[prevFrameIdx]->depth,
                                    frames[prevFrameIdx]->pyramidNormalsMask[0],
                                    0.003, corresps,
                                    frames[currFrameIdx]->pyramidNormals[0],
                                    frames[prevFrameIdx]->pyramidNormals[0],
                                    frames[currFrameIdx]->image,
                                    frames[prevFrameIdx]->image,
                                    maxColorDiff);

            for(int v0 = 0; v0 < corresps.rows; v0++)
            {
                for(int u0 = 0; u0 < corresps.cols; u0++)
                {
                    int c = corresps.at<int>(v0, u0);
                    if(c != -1)
                        correspsCounts.at<int>(v0,u0)++;
                }
            }
        }

        for(int v0 = 0; v0 < correspsCounts.rows; v0++)
        {
            for(int u0 = 0; u0 < correspsCounts.cols; u0++)
            {
                if(correspsCounts.at<int>(v0,u0) < minCorrespCount)
                {
                    curCloud.at<Point3f>(v0,u0) = Point3f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());
                }

                curDepth.at<float>(v0,u0) = curCloud.at<Point3f>(v0,u0).z;
            }
        }
    }

    for(size_t i = 0; i < frames.size(); i++)
    {
        Ptr<RgbdFrame> fr = new RgbdFrame(_frames[i]->image, frames[i]->depth, frames[i]->pyramidMask[0],
                                          frames[i]->normals, _frames[i]->ID);
        _frames[i] = fr;
    }
}
