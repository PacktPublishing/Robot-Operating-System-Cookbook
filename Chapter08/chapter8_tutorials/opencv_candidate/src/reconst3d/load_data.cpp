#include <dirent.h>
#include <iostream>
#include <fstream>
#include <algorithm>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv_candidate_reconst3d/reconst3d.hpp>

using namespace std;
using namespace cv;

static void readDirectory(const string& directoryName, vector<string>& filenames, bool addDirectoryName)
{
    filenames.clear();

    DIR* dir = opendir(directoryName.c_str());
    if(dir != NULL)
    {
        struct dirent* dent;
        while((dent = readdir(dir)) != NULL)
        {
            if(addDirectoryName)
                filenames.push_back(directoryName + "/" + string(dent->d_name));
            else
                filenames.push_back(string(dent->d_name));
        }
    }
    sort(filenames.begin(), filenames.end());
}

void readFrameIndices(const std::string& dirname, std::vector<std::string>& frameIndices)
{
    vector<string> allFilenames;
    readDirectory(dirname, allFilenames, false);

    frameIndices.reserve(allFilenames.size());
    for(size_t i = 0; i < allFilenames.size(); i++)
    {
        const string& imageFilename = allFilenames[i];
        // image_* and .png is at least 11 character
        if(imageFilename.size() < 11)
          continue;

        const string imageIndex = imageFilename.substr(6, imageFilename.length() - 6 - 4);

        if(imageFilename.substr(0, 6) == "image_" &&
           imageIndex.find_first_not_of("0123456789") == std::string::npos &&
           imageFilename.substr(imageFilename.length() - 4, 4) == ".png")
        {
            frameIndices.push_back(imageIndex);
        }
    }
}

void loadFrameData(const std::string& dirname, const std::string& frameIndex, cv::Mat& bgrImage, cv::Mat& depth32F)
{
    string imageFilename = "image_" + frameIndex + ".png";

    cout << "Load " << imageFilename << endl;

    // read image
    {
        string imagePath = dirname + "/" + imageFilename;
        bgrImage = imread(imagePath);
        CV_Assert(!bgrImage.empty());
    }

    // read depth
    {
        string depthPath = "depth_image_" + frameIndex + ".xml.gz";
        FileStorage fs(dirname + "/" + depthPath, FileStorage::READ);
        if(fs.isOpened())
        {
            fs["depth_image"] >> depth32F;
        }
        else
        {
            depthPath = "depth_" + frameIndex + ".png";
            Mat depth = imread(dirname + "/" + depthPath, -1);
            CV_Assert(!depth.empty());
            depth.convertTo(depth32F, CV_32FC1, 0.001);
            depth32F.setTo(std::numeric_limits<float>::quiet_NaN(), depth32F == 0);
        }

        CV_Assert(!depth32F.empty());
        CV_Assert(depth32F.type() == CV_32FC1);
#if 0
        const double depth_sigma = 0.003;
        const double space_sigma = 3.5;  // in pixels
        Mat invalidDepthMask = (depth32F != depth32F) | depth32F;
        depth32F.setTo(-5*depth_sigma, invalidDepthMask);
        Mat filteredDepth;
        bilateralFilter(depth32F, filteredDepth, -1, depth_sigma, space_sigma);
        filteredDepth.setTo(std::numeric_limits<float>::quiet_NaN(), invalidDepthMask);
        depth32F = filteredDepth;
#endif

    }
}


void loadTODLikeBase(const string& dirname, vector<Mat>& bgrImages, vector<Mat>& depthes32F, vector<string>* imageFilenames)
{
    CV_Assert(!dirname.empty());

    vector<string> imageIndices;
    readFrameIndices(dirname, imageIndices);

    bgrImages.resize(imageIndices.size());
    depthes32F.resize(imageIndices.size());
    if(imageFilenames)
        imageFilenames->resize(imageIndices.size());

#pragma omp parallel for
    for(size_t i = 0; i < imageIndices.size(); i++)
    {
        string imageFilename = "image_" + imageIndices[i] + ".png";
        if(imageFilenames)
            (*imageFilenames)[i] = imageFilename;

        loadFrameData(dirname, imageIndices[i], bgrImages[i], depthes32F[i]);
    }
}
