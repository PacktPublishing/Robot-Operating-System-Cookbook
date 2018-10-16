#include <opencv_candidate_reconst3d/reconst3d.hpp>
#include <opencv2/core/core.hpp>

// TODO: remove this fix when it'll became available in OpenCV

#define CV_INIT_ALGORITHM_FIX(classname, algname, memberinit) \
    static ::cv::Algorithm* create##classname() \
    { \
        return new classname; \
    } \
    \
    static ::cv::AlgorithmInfo& classname##_info() \
    { \
        static ::cv::AlgorithmInfo classname##_info_var(algname, create##classname); \
        return classname##_info_var; \
    } \
    \
    static ::cv::AlgorithmInfo& classname##_info_auto = classname##_info(); \
    \
    ::cv::AlgorithmInfo* classname::info() const \
    { \
        static volatile bool initialized = false; \
        \
        if( !initialized ) \
        { \
            initialized = true; \
            classname obj; \
            memberinit; \
        } \
        return &classname##_info(); \
    }

CV_INIT_ALGORITHM_FIX(TableMasker, "ModelCapture.TableMasker",
    obj.info()->addParam(obj, "zFilterMin", obj.zFilterMin);
    obj.info()->addParam(obj, "zFilterMax", obj.zFilterMax);
    obj.info()->addParam(obj, "minTablePart", obj.minTablePart);
    obj.info()->addParam(obj, "minOverlapRatio", obj.minOverlapRatio);
    obj.info()->addParam(obj, "erodeIters", obj.erodeIters);
    obj.info()->addParam(obj, "minObjectPartArea", obj.minObjectPartArea);
    obj.info()->addParam(obj, "cameraMatrix", obj.cameraMatrix);)


CV_INIT_ALGORITHM_FIX(CircularCaptureServer, "ModelCapture.CircularCaptureServer",
    obj.info()->addParam(obj, "tableMasker", obj.tableMasker);
    obj.info()->addParam(obj, "odometry", obj.odometry);
    obj.info()->addParam(obj, "cameraMatrix", obj.cameraMatrix);
    obj.info()->addParam(obj, "maxCorrespColorDiff", obj.maxCorrespColorDiff);
    obj.info()->addParam(obj, "maxCorrespDepthDiff", obj.maxCorrespDepthDiff);
    obj.info()->addParam(obj, "minInliersRatio", obj.minInliersRatio);
    obj.info()->addParam(obj, "skippedTranslation", obj.skippedTranslation);
    obj.info()->addParam(obj, "minTranslationDiff", obj.minTranslationDiff);
    obj.info()->addParam(obj, "maxTranslationDiff", obj.maxTranslationDiff);
    obj.info()->addParam(obj, "minRotationDiff", obj.minRotationDiff);
    obj.info()->addParam(obj, "maxRotationDiff", obj.maxRotationDiff);
    obj.info()->addParam(obj, "isTrajectoryBroken", obj.isTrajectoryBroken);
    obj.info()->addParam(obj, "isInitialied", obj.isInitialied, true);
    obj.info()->addParam(obj, "isFinalized", obj.isFinalized, true);
    obj.info()->addParam(obj, "isLoopClosed", obj.isLoopClosed, true);)

CV_INIT_ALGORITHM_FIX(Feature2dPoseEstimator, "ModelCapture.Feature2dPoseEstimator",
    obj.info()->addParam(obj, "minInliersCount", obj.minInliersCount);
    obj.info()->addParam(obj, "reliableInliersCount", obj.reliableInliersCount);
    obj.info()->addParam(obj, "ransacMaxIterCount", obj.ransacMaxIterCount);
    obj.info()->addParam(obj, "maxPointsDist2d", obj.maxPointsDist2d);
    obj.info()->addParam(obj, "maxDistDiff3d", obj.maxDistDiff3d);
    obj.info()->addParam(obj, "cameraMatrix", obj.cameraMatrix);)

CV_INIT_ALGORITHM_FIX(ArbitraryCaptureServer, "ModelCapture.ArbitraryCaptureServer",
    obj.info()->addParam(obj, "tableMasker", obj.tableMasker);
    obj.info()->addParam(obj, "odometry", obj.odometry);
    obj.info()->addParam(obj, "cameraMatrix", obj.cameraMatrix);
    obj.info()->addParam(obj, "skippedTranslation", obj.skippedTranslation);
    obj.info()->addParam(obj, "minTranslationDiff", obj.minTranslationDiff);
    obj.info()->addParam(obj, "minRotationDiff", obj.minRotationDiff);
    obj.info()->addParam(obj, "isInitialied", obj.isInitialied, true);
    obj.info()->addParam(obj, "isFinalized", obj.isFinalized, true);)


CV_INIT_ALGORITHM_FIX(ModelReconstructor, "ModelCapture.ModelReconstructor",
    obj.info()->addParam(obj, "isShowStepResults", obj.isShowStepResults);
    obj.info()->addParam(obj, "isEstimateRefinedTablePlane", obj.isEstimateRefinedTablePlane);
    obj.info()->addParam(obj, "maxBAPosesCount", obj.maxBAPosesCount);)
