#ifndef RECONST3D_HPP
#define RECONST3D_HPP

#include <opencv2/rgbd/rgbd.hpp>
#include <opencv2/features2d/features2d.hpp>

// Read frames from TOD-like base
void readFrameIndices(const std::string& dirname, std::vector<std::string>& frameIndices);

void loadFrameData(const std::string& dirname, const std::string& frameIndex, cv::Mat& bgrImage, cv::Mat& depth32F);

void loadTODLikeBase(const std::string& dirname, std::vector<cv::Mat>& bgrImages,
                     std::vector<cv::Mat>& depthes32F, std::vector<std::string>* imageFilenames=0);

// TODO remove the following functions from reconst3d API
inline
cv::Point3f rotatePoint(const cv::Point3f& point, const cv::Mat& Rt)
{
    CV_Assert(Rt.type() == CV_64FC1);
    const double * Rt_ptr = Rt.ptr<double>();

    cv::Point3f rotatedPoint;
    rotatedPoint.x = point.x * Rt_ptr[0] + point.y * Rt_ptr[1] + point.z * Rt_ptr[2];
    rotatedPoint.y = point.x * Rt_ptr[4] + point.y * Rt_ptr[5] + point.z * Rt_ptr[6];
    rotatedPoint.z = point.x * Rt_ptr[8] + point.y * Rt_ptr[9] + point.z * Rt_ptr[10];
    return rotatedPoint;
}

inline
cv::Point3f translatePoint(const cv::Point3f& point, const cv::Mat& Rt)
{
    CV_Assert(Rt.type() == CV_64FC1);
    const double * Rt_ptr = Rt.ptr<double>();

    cv::Point3f translatedPoint;
    translatedPoint.x = point.x + Rt_ptr[3];
    translatedPoint.y = point.y + Rt_ptr[7];
    translatedPoint.z = point.z + Rt_ptr[11];
    return translatedPoint;
}

inline
cv::Point3f transformPoint(const cv::Point3f& point, const cv::Mat& Rt)
{
    return translatePoint(rotatePoint(point, Rt), Rt);
}

// Find a table mask
class TableMasker: public cv::Algorithm
{
public:
    static double DEFAULT_Z_FILTER_MIN() {return 0.005;}
    static double DEFAULT_Z_FILTER_MAX() {return 0.5;}
    static double DEFAULT_MIN_TABLE_PART() {return 0.1;}
    static double DEFAULT_MIN_OVERLAP_RATIO() {return 0.6;}
    static const int DEFAULT_ERODE_ITERS = 10;
    static const int DEFAULT_MIN_OBJECT_PART_AREA = 15;

    TableMasker();
    bool operator()(const cv::Mat& cloud, const cv::Mat& normals,
                    cv::Mat& tableWithObjectMask, cv::Mat* objectMask=0, cv::Vec4f* planeCoeffs=0) const;

    bool operator()(const cv::Mat& cloud, const cv::Mat& normals, const cv::Mat& prevTableMask,
                    cv::Mat& tableMask, cv::Mat& objectMask, cv::Vec4f* planeCoeffs=0) const;

    cv::AlgorithmInfo*
    info() const;

protected:
    int findTablePlane(const cv::Mat& cloud, const cv::Mat& prevMask,
                       const cv::Mat_<uchar>& planesMask, size_t planesCount,
                       cv::Mat& tableMask) const;
    cv::Mat calcObjectMask(const cv::Mat& cloud,
                           const cv::Mat& tableMask, const cv::Vec4f& tableCoeffitients) const;


    mutable cv::Ptr<cv::RgbdPlane> planeComputer;

    double zFilterMin;
    double zFilterMax;
    double minTablePart;
    double minOverlapRatio;
    int erodeIters;
    int minObjectPartArea;

    cv::Mat cameraMatrix;
};

struct PosesLink
{
    PosesLink(int srcIndex=-1, int dstIndex=-1, const cv::Mat& Rt=cv::Mat());
    int srcIndex;
    int dstIndex;
    cv::Mat Rt; // optional (for loop closure)
};

struct TrajectoryFrames
{
    enum { VALIDFRAME = 1,
           KEYFRAME   = VALIDFRAME | 2,
           DEFAULT    = VALIDFRAME
         };

    void push(const cv::Ptr<cv::RgbdFrame>& frame, const cv::Mat& pose,
              const cv::Mat& objectMask, int state);
    void clear();

    void save(const std::string& filename) const;
    void load(const std::string& filename);

    int resumeFrameState;
    std::vector<cv::Ptr<cv::RgbdFrame> > frames;
    std::vector<int> frameStates;
    std::vector<cv::Mat> objectMasks;
    std::vector<cv::Mat> poses;
    std::vector<PosesLink> keyframePosesLinks;
};

class CircularCaptureServer: public cv::Algorithm
{
public:
    struct FramePushOutput
    {
        FramePushOutput();

        int frameState;
        cv::Ptr<cv::OdometryFrame> frame;
        cv::Mat pose;
        cv::Mat objectMask;
    };

    static const int DEFAULT_MAX_CORRESP_COLOR_DIFF = 50; // it's rough now, because first and last frame may have large changes of light conditions
                                                          // TODO: do something with light changes
    static double DEFAULT_MAX_CORRESP_DEPTH_DIFF() {return 0.01;} // meters
    static double DEFAULT_MIN_INLIERS_RATIO() {return 0.6;}
    static double DEFAULT_SKIPPED_TRANSLATION() {return 0.4;} //meters
    static double DEFAULT_MIN_TRANSLATION_DIFF() {return 0.08;} //meters
    static double DEFAULT_MAX_TRANSLATION_DIFF() {return 0.3;} //meters
    static double DEFAULT_MIN_ROTATION_DIFF() {return 10;} //degrees
    static double DEFAULT_MAX_ROTATION_DIFF() {return 30;} //degrees

    CircularCaptureServer();

    cv::Ptr<FramePushOutput> push(const cv::Mat& image, const cv::Mat& depth, int frameID);

    void initialize(const cv::Size& frameResolution, int storeFramesWithState=TrajectoryFrames::KEYFRAME);

    void reset();

    cv::Ptr<TrajectoryFrames> finalize();

    cv::AlgorithmInfo*
    info() const;

protected:
    void filterImage(const cv::Mat& src, cv::Mat& dst) const;
    void firterDepth(const cv::Mat& src, cv::Mat& dst) const;

    // used algorithms
    cv::Ptr<cv::RgbdNormals> normalsComputer; // inner only
    cv::Ptr<TableMasker> tableMasker;
    cv::Ptr<cv::Odometry> odometry;

    // output keyframes data
    cv::Ptr<TrajectoryFrames> trajectoryFrames;

    // params
    cv::Mat cameraMatrix;

    int maxCorrespColorDiff;
    double maxCorrespDepthDiff;

    double minInliersRatio;
    double skippedTranslation;
    double minTranslationDiff;
    double maxTranslationDiff;
    double minRotationDiff;
    double maxRotationDiff;

    // state variables
    cv::Ptr<cv::OdometryFrame> firstKeyframe, lastKeyframe, prevFrame, closureFrame;
    cv::Mat prevPose;
    int prevFrameID;

    bool isTrajectoryBroken;
    bool isLoopClosing, isLoopClosed;
    double translationSum;
    float closureInliersRatio;
    int closureFrameID;
    bool isClosureFrameKey;
    cv::Mat closureBgrImage;
    cv::Mat closureObjectMask;
    cv::Mat closurePose, closurePoseWithFirst;

    bool isInitialied, isFinalized;
};

class Feature2dPoseEstimator : public cv::Algorithm
{
public:
    static const int DEFAULT_MIN_INLIERS_COUNT = 20;
    static const int DEFAULT_RELIABLE_INLIERS_COUNT = 1000;
    static const int DEFAULT_RANSAC_MAX_ITER_COUNT = 20000;
    static double DEFAULT_MAX_POINTS_DIST2D(){return 3.;}
    static double DEFAULT_MAX_DIST_DIFF3D(){return 0.01;}

    Feature2dPoseEstimator();

    cv::Mat operator()(const std::vector<cv::KeyPoint>& srcKeypoints, const cv::Mat& srcDescriptors, const cv::Mat& srcCloud,
                       const std::vector<cv::KeyPoint>& dstKeypoints, const cv::Mat& dstDescriptors, const cv::Mat& dstCloud,
                       std::vector<cv::DMatch>* matches=0) const;
    cv::AlgorithmInfo*
    info() const;

protected:
    cv::Mat estimateRt(const std::vector<cv::KeyPoint>& srcKeypoints, const std::vector<cv::KeyPoint>& dstKeypoints,
                       const cv::Mat& srcCloud, const cv::Mat& dstCloud,
                       std::vector<cv::DMatch>& matches) const;

    int minInliersCount;
    int reliableInliersCount;
    int ransacMaxIterCount;
    double maxPointsDist2d;
    double maxDistDiff3d;
    cv::Mat cameraMatrix;
};

class ArbitraryCaptureServer: public cv::Algorithm
{
public:
    struct FramePushOutput
    {
        FramePushOutput();

        bool isKeyframe;
        cv::Mat pose;
    };

    static const int DEFAULT_MIN_OBJECT_SIZE = 50; //pixels
    static double DEFAULT_MIN_TRANSLATION_DIFF() {return CircularCaptureServer::DEFAULT_MIN_TRANSLATION_DIFF();} //meters
    static double DEFAULT_MIN_ROTATION_DIFF() {return CircularCaptureServer::DEFAULT_MIN_ROTATION_DIFF();} //degrees

    ArbitraryCaptureServer();

    FramePushOutput push(const cv::Mat& image, const cv::Mat& depth, int frameID);

    void initialize(const cv::Size& frameResolution);

    void reset();

    cv::Ptr<TrajectoryFrames> finalize();

    cv::AlgorithmInfo*
    info() const;

protected:
    struct TrajectorySegment
    {
        TrajectorySegment();
        void push(const cv::Ptr<cv::OdometryFrame>& frame, const cv::Mat& pose,
                  const cv::Mat& objectMask, const cv::Mat& tableMask,
                  const cv::Vec4f& tableCoeff, const cv::Mat& bgrImage);

        std::vector<cv::Ptr<cv::OdometryFrame> > frames;
        std::vector<cv::Mat> poses;

        std::vector<cv::Mat> bgrImages;
        std::vector<cv::Mat> objectMasks;
        std::vector<cv::Mat> tableMasks;

        std::vector<cv::Vec4f> tableCoeffs;

        int representFrameIndex;
        std::vector<cv::KeyPoint> representFrameKeypoints;
        cv::Mat representFrameDescriptors;

        cv::Ptr<cv::OdometryFrame> lastFrame;
        cv::Mat lastPose;

        bool isFinalized;
    };

    struct Feature2dEdge
    {
        Feature2dEdge(int srcSegmentIndex=-1, int srcFrameIndex=-1,
                      int dstSegmentIndex=-1, int dstFrameIndex=-1,
                      int inliersCount=0, const cv::Mat& Rt=cv::Mat());
        int srcSegmentIndex;
        int srcFrameIndex;
        int dstSegmentIndex;
        int dstFrameIndex;
        int inliersCount;
        cv::Mat Rt;
    };

    cv::Ptr<TrajectorySegment> getActiveSegment() const;
    void estimateFeatures2dEdges(int srcSegmentIndex, int srcFrameIndex,
                                 const std::vector<cv::KeyPoint>& srcKeypoints, const cv::Mat& srcDescriptors, const cv::Mat& srcCloud,
                                 std::vector<Feature2dEdge>& edges) const;
    void finalizeLastSegment();
    cv::Ptr<TrajectorySegment> createNewSegment();

    // used algorithms
    cv::Ptr<cv::RgbdNormals> normalsComputer; // inner only
    cv::Ptr<TableMasker> tableMasker;
    cv::Ptr<cv::Odometry> odometry;
    cv::Ptr<cv::Feature2D> featureComputer;
    cv::Ptr<Feature2dPoseEstimator> feature2dPoseEstimator;

    // params
    cv::Mat cameraMatrix;
    double skippedTranslation;
    double minTranslationDiff;
    double minRotationDiff;
    int minObjectSize;

    // state variables
    cv::Mat prevTableMask;
    std::vector<cv::Ptr<TrajectorySegment> > trajectorySegments;
    std::vector<Feature2dEdge> feature2dEdges;

    bool isInitialied, isFinalized;
};

class ObjectModel
{
public:
    ObjectModel();
    ObjectModel(const std::vector<cv::Ptr<cv::RgbdFrame> >& frames, const std::vector<cv::Mat>& poses,
                const cv::Mat& cameraMatrix, const std::vector<int>& frameIndices=std::vector<int>());
    void create(const std::vector<cv::Ptr<cv::RgbdFrame> >& frames, const std::vector<cv::Mat>& poses,
                const cv::Mat& cameraMatrix, const std::vector<int>& frameIndices=std::vector<int>());

    void clear();

    void read_ply(const std::string& filename);
    void write_ply(const std::string& filename) const;

    void show(float gridSize=0.001f, bool withCameraPoses=false, int normalLevel=0) const;

    std::vector<cv::Vec3b> colors;
    std::vector<cv::Point3f> points3d;
    std::vector<cv::Point3f> normals;

    // TODO: poses should not be here?
    std::vector<cv::Mat> cameraPoses;
    cv::Mat tablePlane;
};

class ModelReconstructor : public cv::Algorithm
{
public:
    static const int DEFAULT_MAX_BA_POSES_COUNT = -1; // use all keyframes
    ModelReconstructor();

    void reconstruct(const cv::Ptr<TrajectoryFrames>& trajectoryFrames, const cv::Mat& cameraMatrix, cv::Ptr<ObjectModel>& model) const;

    cv::AlgorithmInfo*
    info() const;

private:
    // TODO make more algorithm params available outside

    bool isShowStepResults;
    int isEstimateRefinedTablePlane;
    int maxBAPosesCount;
};

#endif // RECONST3D_HPP
