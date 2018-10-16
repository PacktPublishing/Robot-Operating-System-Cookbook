/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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
 Author: Vincent Rabaud
 *********************************************************************/

#ifndef FEATURE2D_HPP_
#define FEATURE2D_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace feature2d
{

class AffineAdaptedFeature2D : public cv::Feature2D
{
public:
    AffineAdaptedFeature2D(const cv::Ptr<cv::Feature2D>& feature2d);
    AffineAdaptedFeature2D(const cv::Ptr<cv::FeatureDetector>& featureDetector,
                           const cv::Ptr<cv::DescriptorExtractor>& descriptorExtractor);

    virtual int descriptorSize() const;
    virtual int descriptorType() const;

    virtual void operator()(cv::InputArray image, cv::InputArray mask,
                            std::vector<cv::KeyPoint>& keypoints,
                            cv::OutputArray descriptors,
                            bool useProvidedKeypoints=false) const;

protected:
    void initialize();

    void detectAndComputeImpl(const cv::Mat& image, const cv::Mat& mask, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) const;

    void computeImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) const;
    void detectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat()) const;

    cv::Ptr<cv::Feature2D> feature2d;
    cv::Ptr<cv::FeatureDetector> featureDetector;
    cv::Ptr<cv::DescriptorExtractor> descriptorExtractor;

    std::vector<cv::Vec2f> affineTransformParams;
};

}

#endif
