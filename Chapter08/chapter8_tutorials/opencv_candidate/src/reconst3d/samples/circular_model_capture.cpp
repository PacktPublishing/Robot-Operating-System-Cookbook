#include <iostream>

#include <opencv_candidate_reconst3d/reconst3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

static Mat defaultCameraMatrix()
{
    double vals[] = {525., 0., 3.1950000000000000e+02,
                    0., 525., 2.3950000000000000e+02,
                    0., 0., 1.};
    return Mat(3,3,CV_64FC1,vals).clone();
}

int main(int argc, char** argv)
{
    if(argc != 3  && argc != 4)
    {
        cout << "Format: " << argv[0] << " train_dirname construct_if_loop_closure_only [model_filename]" << endl;
        cout << "   train_dirname - a path to the directory with TOD-like training base." << endl;
        cout << "   construct_if_loop_closure_only - 0 or 1. If 1 the model reconstruction will be run if the loop closure was detected. If 0,"
                " it will be run in any case (and result can be not very accurate in this case)." << endl;
        cout << "   model_filename - an optional parameter, it's a filename that will be used to save trained model." << endl;
        return -1;
    }

    // Load the data
    const string dirname = argv[1];
    bool constructIfLoopClosureOnly = true;

    if(string(argv[2]) == "0")
        constructIfLoopClosureOnly = false;
    else if(string(argv[2]) != "1")
    {
        cout << "Incorrect second parameter passed to the application." << endl;
        return -1;
    }

    Mat cameraMatrix = defaultCameraMatrix();

    vector<string> frameIndices;
    readFrameIndices(dirname, frameIndices);
    if(frameIndices.empty())
    {
        cout << "Can not load the data from given directory of the base: " << dirname << endl;
        return -1;
    }

    cout << "Frame indices count " << frameIndices.size() << endl;

    CircularCaptureServer onlineCaptureServer;
    onlineCaptureServer.set("cameraMatrix", cameraMatrix);
    onlineCaptureServer.initialize(Size(640,480));
    for(size_t i = 0; i < frameIndices.size(); i++)
    {
        Mat bgrImage, depth;
        loadFrameData(dirname, frameIndices[i], bgrImage, depth);

        onlineCaptureServer.push(bgrImage, depth, i);
        if(onlineCaptureServer.get<bool>("isLoopClosed"))
            break;
        if(onlineCaptureServer.get<bool>("isFinalized"))
        {
            cout << "The trajecotry construction is finalized ahead of time.";
            return -1;
        }
    }

    Ptr<TrajectoryFrames> trajectoryFrames = onlineCaptureServer.finalize();
    if(constructIfLoopClosureOnly && !onlineCaptureServer.get<bool>("isLoopClosed"))
        return -1;

    ModelReconstructor reconstructor;
    reconstructor.set("isShowStepResults", true);
    reconstructor.set("isEstimateRefinedTablePlane", true);

    Ptr<ObjectModel> model;
    reconstructor.reconstruct(trajectoryFrames, cameraMatrix, model);

    if(argc == 4)
    {
        model->write_ply(argv[3]);
#if 0
        FileStorage fs(string(argv[3]) + ".tablePlane.xml", FileStorage::WRITE);
        CV_Assert(fs.isOpened());
        fs << "tablePlane" << model->tablePlane;
#endif
    }

    model->show();

    return 0;
}
