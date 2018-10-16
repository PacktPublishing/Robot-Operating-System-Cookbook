/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <opencv_creative/reader.h>

#include <opencv2/highgui/highgui.hpp>

#include <iostream>

int
main(int argc, char ** argv)
{
  cv::VideoWriter video_writer("video.mpg", CV_FOURCC('P', 'I', 'M', '1'), 30, cv::Size(320, 240), false);
  creative::Reader reader;
  creative::Reader::setImageTypes(creative::Reader::COLOR + creative::Reader::DEPTH + creative::Reader::POINTS3D);

  creative::Reader::initialize();
  cv::namedWindow("color", 0);
  cv::namedWindow("depth", 0);
  for (size_t index = 0; index < 30*30; ++index)
  {
    std::vector<cv::Mat> images;
    creative::Reader::getImages(images);
    cv::Mat color, depth, points3d;
    color = images[0];
    depth = images[1];
    points3d = images[2];
    std::cout << points3d.cols;
    if (!color.empty())
      cv::imshow("color", color);

    if (!depth.empty())
    {
      cv::Mat visible_depth;
      // 500mm is 255
      // 100mm is 0
      depth.convertTo(visible_depth, CV_8U, 255./400., -100);
      video_writer << visible_depth;
      cv::imshow("depth", visible_depth);
    }

    if ((!color.empty()) || (!depth.empty()))
      cv::waitKey(10);
  }
}
