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

#ifndef CREATIVE_H_
#define CREATIVE_H_

#include <boost/thread.hpp>

#include <opencv2/core/core.hpp>

namespace DepthSense
{
  class Context;
  class ColorNode;
  class DepthNode;
}

namespace creative
{
  class ReaderImpl;

  class Reader
  {
  public:
    enum IMAGE_TYPE
    {
      COLOR = 1, DEPTH = 2, POINTS3D = 4
    };
    Reader();

    ~Reader();

    /** Start the capture thread. This can be called beforehand to make sure data is synchronized
     * as color is started and the depth node is registered some time after (otherwise, everything crashes)
     */
    static void
    initialize();

    /** Set the image types that should be returned by getImages
     * @param all_images it should be the sum of chosen IMAGE_TYPE
     */
    static void
    setImageTypes(int all_images);

    /** Returns whether the image type image_type is returned when calling getImages
     * @param image_type
     * @return
     */
    static bool
    hasImageType(IMAGE_TYPE image_type);

    /** Returns the status of the reader
     * @return
     */
    static bool
    isInitialized();

    /** Return the current images. There is only one interface that retrieves all
     * images to ensure synchronization
     * @param images the images that were chosen in setImageTypes will be put in that vector, after clearing it
     */
    static void
    getImages(std::vector<cv::Mat> &images);

  private:
    static ReaderImpl *impl_;
    static size_t count_;
  };
}

#endif /* CREATIVE_H_ */
