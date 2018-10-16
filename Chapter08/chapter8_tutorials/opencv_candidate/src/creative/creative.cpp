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

#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include <opencv_creative/reader.h>

#include <DepthSense.hxx>

#include <iostream>

namespace creative
{
  void
  getAvailableNodes(DepthSense::Context context, DepthSense::ColorNode &color_node, DepthSense::DepthNode &depth_node)
  {
    // obtain the list of devices attached to the host
    std::vector<DepthSense::Device> devices = context.getDevices();
    for (std::vector<DepthSense::Device>::const_iterator iter = devices.begin(); iter != devices.end(); iter++)
    {
      DepthSense::Device device = *iter;
      // obtain the list of nodes of the current device
      std::vector<DepthSense::Node> nodes = device.getNodes();
      for (std::vector<DepthSense::Node>::const_iterator nodeIter = nodes.begin(); nodeIter != nodes.end(); nodeIter++)
      {
        DepthSense::Node node = *nodeIter;
        if (!color_node.isSet())
          color_node = node.as<DepthSense::ColorNode>();
        if (!depth_node.isSet())
          depth_node = node.as<DepthSense::DepthNode>();
      }
      break;
    }
    // return an unset color node
    if (!color_node.isSet())
      color_node = DepthSense::ColorNode();
    if (!depth_node.isSet())
      depth_node = DepthSense::DepthNode();
  }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /** Class where all the magic happens to interace with the Creative camera
   * @return
   */
  class ReaderImpl
  {
  public:
    ReaderImpl()
    {
    }

    ~ReaderImpl()
    {
      context_.stopNodes();
      while (!context_.getRegisteredNodes().empty()) {
        context_.unregisterNode(context_.getRegisteredNodes().back());
      }
      context_.quit();
    }

    static void
    onNewColorSample(DepthSense::ColorNode obj, DepthSense::ColorNode::NewSampleReceivedData data)
    {
      // Read the color buffer and display
      int32_t w, h;
      DepthSense::FrameFormat_toResolution(data.captureConfiguration.frameFormat, &w, &h);

      cv::Mat color_yuy2(h, w, CV_8UC2, const_cast<void*>((const void*) (data.colorMap)));
      {
        boost::unique_lock<boost::mutex> lock(color_mutex_);
        cv::cvtColor(color_yuy2, color_, CV_YUV2BGR_YUY2);
      }
      color_cond_.notify_all();

      if (image_types_ > 1)
      {
        static size_t index = 0;
        // VERY HACKISH: but wait for another N frames (totally arbitrary) before getting a depth node in there
        if (index < 10)
          ++index;
        else if (index == 10)
        {
          all_nodes_are_up_ = true;
          context_.registerNode(depth_node_);
          ++index;
        }
      }
    }

    static void
    onNewDepthSample(DepthSense::DepthNode obj, DepthSense::DepthNode::NewSampleReceivedData data)
    {
      // Read the color buffer and display
      int32_t w, h;
      DepthSense::FrameFormat_toResolution(data.captureConfiguration.frameFormat, &w, &h);
      cv::Mat depth_single(h, w, CV_16UC1, const_cast<void*>((const void*) (data.depthMap)));
      {
        boost::unique_lock<boost::mutex> lock(color_mutex_);
        depth_single.copyTo(depth_);
      }
      depth_cond_.notify_all();
    }

    static void
    initialize()
    {
      if (is_initialized_)
        return;

      // create a connection to the DepthSense server at localhost
      context_ = DepthSense::Context::create();
      // get the first available color sensor
      getAvailableNodes(context_, color_node_, depth_node_);

      // enable the capture of the color map
      // Get RGB data
      context_.requestControl(color_node_);
      color_node_.setEnableColorMap(true);
      DepthSense::ColorNode::Configuration color_configuration(DepthSense::FRAME_FORMAT_QVGA, 30,
                                                               DepthSense::POWER_LINE_FREQUENCY_50HZ,
                                                               DepthSense::COMPRESSION_TYPE_YUY2);
      color_node_.setConfiguration(color_configuration);
      context_.releaseControl(color_node_);

      // Get depth data
      context_.requestControl(depth_node_);
      if (hasImageType(Reader::DEPTH))
        depth_node_.setEnableDepthMap(true);
      if (hasImageType(Reader::POINTS3D))
        depth_node_.setEnableVerticesFloatingPoint(true);
      DepthSense::DepthNode::DepthNode::Configuration depth_configuration(DepthSense::FRAME_FORMAT_QVGA, 30,
                                                                          DepthSense::DepthNode::CAMERA_MODE_CLOSE_MODE,
                                                                          true);
      depth_node_.setConfiguration(depth_configuration);
      depth_node_.setConfidenceThreshold(50);
      context_.releaseControl(depth_node_);

      // connect a callback to the newSampleReceived event of the color node
      color_node_.newSampleReceivedEvent().connect(ReaderImpl::onNewColorSample);
      depth_node_.newSampleReceivedEvent().connect(ReaderImpl::onNewDepthSample);

      // If only one node to register, great go ahead
      if (hasImageType(Reader::COLOR))
      {
        context_.registerNode(color_node_);
        all_nodes_are_up_ = (image_types_ == 1);
      }
      else
      {
        all_nodes_are_up_ = true;
        context_.registerNode(depth_node_);
      }
      context_.startNodes();

      // Spawn the thread that will just run
      thread_ = boost::thread(run);

      is_initialized_ = true;
    }

    static void
    setImageTypes(int image_types)
    {
      image_types_ = image_types;
    }

    static bool
    hasImageType(Reader::IMAGE_TYPE image_type)
    {
      return (image_types_ & image_type);
    }

    void
    getImages(std::vector<cv::Mat> &images) const
    {
      images.clear();
      if (hasImageType(Reader::COLOR))
      {
        {
          boost::unique_lock<boost::mutex> lock(color_mutex_);
          color_cond_.wait(lock);
        }
        images.resize(images.size() + 1);
        color_.copyTo(images.back());
      }

      if ((hasImageType(Reader::DEPTH)) || (hasImageType(Reader::POINTS3D)))
      {
        boost::unique_lock<boost::mutex> lock(depth_mutex_);
        depth_cond_.wait(lock);
        if (hasImageType(Reader::DEPTH))
        {
          images.resize(images.size() + 1);
          depth_.copyTo(images.back());
        }
        if (hasImageType(Reader::POINTS3D))
        {
          images.resize(images.size() + 1);
          points3d_.copyTo(images.back());
        }
      }
    }

    static bool is_initialized_;
  private:
    static void
    run()
    {
      context_.run();
    }

    static DepthSense::Context context_;
    static DepthSense::ColorNode color_node_;
    static DepthSense::DepthNode depth_node_;

    /** The thread in which the data will be captured */
    static boost::thread thread_;

    /** The iamges in which to store the different data types */
    static cv::Mat_<cv::Vec3b> color_;
    static cv::Mat_<unsigned short> depth_;
    static cv::Mat_<cv::Vec3f> points3d_;

    /** Variable indicating whether all the nodes are up and register */
    static bool all_nodes_are_up_;

    /** a sum of Reader::IMAGE_TYPE declaring what info is retrieved from the nodes */
    static int image_types_;

    static boost::mutex color_mutex_;
    static boost::mutex depth_mutex_;
    static boost::condition_variable color_cond_;
    static boost::condition_variable depth_cond_;
  };

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ReaderImpl *Reader::impl_ = new ReaderImpl();
  size_t Reader::count_ = 0;
  bool ReaderImpl::is_initialized_ = false;
  DepthSense::Context ReaderImpl::context_;
  boost::thread ReaderImpl::thread_;
  DepthSense::ColorNode ReaderImpl::color_node_;
  DepthSense::DepthNode ReaderImpl::depth_node_;
  cv::Mat_<cv::Vec3b> ReaderImpl::color_;
  cv::Mat_<unsigned short> ReaderImpl::depth_;
  cv::Mat_<cv::Vec3f> ReaderImpl::points3d_;

  bool ReaderImpl::all_nodes_are_up_ = false;
  int ReaderImpl::image_types_ = 0;

  boost::mutex ReaderImpl::color_mutex_;
  boost::mutex ReaderImpl::depth_mutex_;
  boost::condition_variable ReaderImpl::color_cond_;
  boost::condition_variable ReaderImpl::depth_cond_;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  Reader::Reader()
  {
    ++count_;
    if (!impl_)
      impl_ = new ReaderImpl();
  }

  /** Clean all the contexts and unregister the nodes
   */
  Reader::~Reader()
  {
    --count_;
    if (count_ == 0)
      delete impl_;
  }

  void
  Reader::initialize()
  {
    impl_->initialize();
  }

  void
  Reader::setImageTypes(int all_images)
  {
    impl_->setImageTypes(all_images);
  }

  bool
  Reader::hasImageType(IMAGE_TYPE image_type)
  {
    return impl_->hasImageType(image_type);
  }

  bool
  Reader::isInitialized()
  {
    return impl_->is_initialized_;
  }

  void
  Reader::getImages(std::vector<cv::Mat> &images)
  {
    impl_->getImages(images);
  }
}
