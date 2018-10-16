#ifndef RECONST3D_CREATE_OPTIMIZER
#define RECONST3D_CREATE_OPTIMIZER

#include <Eigen/Geometry>

#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/solver.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/robust_kernel_impl.h>

#include <opencv2/core/core.hpp>
#include <opencv2/rgbd/rgbd.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <opencv_candidate_reconst3d/reconst3d.hpp>

typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 3> >  G2OBlockSolver;
typedef g2o::LinearSolver< G2OBlockSolver::PoseMatrixType> G2OLinearSolver;
typedef g2o::LinearSolverCholmod<G2OBlockSolver::PoseMatrixType> G2OLinearCholmodSolver;
typedef g2o::LinearSolverCSparse<G2OBlockSolver::PoseMatrixType> G2OLinearCSparseSolver;

const std::string DEFAULT_LINEAR_SOLVER_TYPE = "cholmod"; //"csparse";
const std::string DEFAULT_NON_LINEAR_SOLVER_TYPE  = "GN";

// get norms
//
static inline
float tvecNorm(const cv::Mat& Rt)
{
    return cv::norm(Rt(cv::Rect(3,0,1,3)));
}

static inline
float rvecNormDegrees(const cv::Mat& Rt)
{
    cv::Mat rvec;
    cv::Rodrigues(Rt(cv::Rect(0,0,3,3)), rvec);
    return cv::norm(rvec) * 180. / CV_PI;
}

// convertions
//
inline
Eigen::Vector3d cvtPoint_ocv2egn(const cv::Point3f& ocv_p)
{
    Eigen::Vector3d egn_p;
    egn_p[0] = ocv_p.x;
    egn_p[1] = ocv_p.y;
    egn_p[2] = ocv_p.z;

    return egn_p;
}

inline
cv::Mat cvtIsometry_egn2ocv(const Eigen::Isometry3d& egn_o)
{
    Eigen::Matrix3d eigenRotation = egn_o.rotation();
    Eigen::Matrix<double,3,1> eigenTranslation = egn_o.translation();

    cv::Mat R, t;
    eigen2cv(eigenRotation, R);
    eigen2cv(eigenTranslation, t);
    t = t.reshape(1,3);

    cv::Mat ocv_o = cv::Mat::eye(4,4,CV_64FC1);
    R.copyTo(ocv_o(cv::Rect(0,0,3,3)));
    t.copyTo(ocv_o(cv::Rect(3,0,1,3)));

    return ocv_o;
}

// corresps
inline
void set2shorts(int& dst, int short_v1, int short_v2)
{
    unsigned short* ptr = reinterpret_cast<unsigned short*>(&dst);
    ptr[0] = static_cast<unsigned short>(short_v1);
    ptr[1] = static_cast<unsigned short>(short_v2);
}

inline
void get2shorts(int src, int& short_v1, int& short_v2)
{
    typedef union { int vint32; unsigned short vuint16[2]; } s32tou16;
    const unsigned short* ptr = (reinterpret_cast<s32tou16*>(&src))->vuint16;
    short_v1 = ptr[0];
    short_v2 = ptr[1];
}

int computeCorrespsFiltered(const cv::Mat& K, const cv::Mat& K_inv, const cv::Mat& Rt,
                            const cv::Mat& depth0, const cv::Mat& validMask0,
                            const cv::Mat& depth1, const cv::Mat& selectMask1, float maxDepthDiff,
                            cv::Mat& corresps,
                            const cv::Mat& normals0, const cv::Mat& normals1,
                            const cv::Mat& image0, const cv::Mat& image1,
                            float maxColorDiff = FLT_MAX);

void selectPosesSubset(const std::vector<cv::Mat>& poses,
                       const std::vector<int>& indices,
                       std::vector<int>& selectedIndices, size_t count);

// create solver
//
inline
G2OLinearSolver* createLinearSolver(const std::string& type)
{
    G2OLinearSolver* solver = 0;
    if(type == "cholmod")
        solver = new G2OLinearCholmodSolver();
    else if(type == "csparse")
    {
        solver = new G2OLinearCSparseSolver();
    }
    else
    {
        CV_Assert(0);
    }

    return solver;
}

inline
G2OBlockSolver* createBlockSolver(G2OLinearSolver* linearSolver)
{
    return new G2OBlockSolver(linearSolver);
}

inline
g2o::OptimizationAlgorithm* createNonLinearSolver(const std::string& type, G2OBlockSolver* blockSolver)
{
    g2o::OptimizationAlgorithm* solver = 0;
    if(type == "GN")
        solver = new g2o::OptimizationAlgorithmGaussNewton(blockSolver);
    else if(type == "LM")
        solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
    else
        CV_Assert(0);

    return solver;
}

inline
g2o::SparseOptimizer* createOptimizer(g2o::OptimizationAlgorithm* solver)
{
    g2o::SparseOptimizer* optimizer = new g2o::SparseOptimizer();
    optimizer->setAlgorithm(solver);
    optimizer->setVerbose(true);
    return optimizer;
}

// graph opt

// Restore refined camera poses from the graph.
void getSE3Poses(g2o::SparseOptimizer* optimizer, const std::vector<int>& frameIndices, std::vector<cv::Mat>& poses);

// Fill the given graph by vertices and edges for the camera pose refinement geometrically.
// Each vertex is a camera pose. Each edge constraint is the odometry between linked vertices.
void fillGraphSE3(g2o::SparseOptimizer* optimizer,
                  const std::vector<cv::Mat>& poses, const std::vector<PosesLink>& posesLinks,
                  std::vector<int>& frameIndices);

// Refine camera poses geometrically
void refineGraphSE3(const std::vector<cv::Mat>& poses, const std::vector<PosesLink>& posesLinks,
                    std::vector<cv::Mat>& refinedPoses, std::vector<int>& frameIndices);

void refineGraphSE3Segment(const std::vector<cv::Mat>& odometryPoses,
                           const std::vector<cv::Mat>& partiallyRefinedPoses,
                           const std::vector<int>& refinedFrameIndices,
                           std::vector<cv::Mat>& refinedAllPoses);



// Graph with 2 types of edges: odometry and Rgbd+ICP for correspondences.
// TODO we need next iteration of the code refactoring
void fillGraphSE3RgbdICP(g2o::SparseOptimizer* optimizer, int pyramidLevel,
                         const std::vector<cv::Ptr<cv::OdometryFrame> >& frames,
                         const std::vector<cv::Mat>& poses, const std::vector<PosesLink>& posesLinks, const cv::Mat& cameraMatrix,
                         std::vector<int>& frameIndices, 
                         double maxTranslation, double maxRotation, double maxDepthDiff);

// Refine camera poses by graph with odometry edges and Rgbd+ICP edges
void refineGraphSE3RgbdICP(const std::vector<cv::Ptr<cv::RgbdFrame> >& frames,
                           const std::vector<cv::Mat>& poses, const std::vector<PosesLink>& posesLinks, const cv::Mat& cameraMatrix,
                           float pointsPart, std::vector<cv::Mat>& refinedPoses, std::vector<int>& frameIndices);


void refineGraphSE3RgbdICPModel(std::vector<cv::Ptr<cv::RgbdFrame> >& frames,
                                const std::vector<cv::Mat>& poses, const std::vector<PosesLink>& posesLinks, const cv::Mat& cameraMatrix,
                                std::vector<cv::Mat>& refinedPoses, std::vector<int>& frameIndices);

#endif
