/* -*- mode: C++ -*- */
/* $Id: 7290b1e8d933f52fa8cbf73baaf239c93a783478 $ */

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2011 Jack O'Quin
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
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
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
*********************************************************************/

#ifndef _WGS84_H_
#define _WGS84_H_

#include <limits>
#include <ctype.h>
#include <geographic_msgs/GeoPoint.h>
#include <geographic_msgs/GeoPose.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/tf.h>

/** @file

    @brief WGS 84 geodetic system for ROS latitude and longitude messages

    Standard ROS latitude and longitude coordinates are defined in
    terms of the World Geodetic System (WGS 84) ellipsoid used by most
    navigation satellite receivers.

    Many other geodetic coordinate systems can be defined.  They
    should always be converted to WGS 84 when publishing ROS messages
    to avoid confusion among subscribers.
    
    @author Jack O'Quin
 */

namespace geodesy
{
  /** Convert any coordinate to any other via intermediate WGS 84
   *  representation.
   *
   *  @author Tully Foote
   *
   *  @note Every coordinate type @b must implement fromMsg() and
   *        toMsg() functions for both points and poses.
   */
  template <class From, class To>
  void convert(const From &from, To &to);

  /** Convert any coordinate to itself. */
  template <class Same>
  void convert(const Same &from, Same &to);

  /** Convert one WGS 84 geodetic point to another.
   *
   *  @param from WGS 84 point message.
   *  @param to another point.
   */
  static inline void fromMsg(const geographic_msgs::GeoPoint &from,
                             geographic_msgs::GeoPoint &to)
  {
    convert(from, to);
  }

  /** Convert one WGS 84 geodetic pose to another.
   *
   *  @param from WGS 84 pose message.
   *  @param to another pose.
   */
  static inline void fromMsg(const geographic_msgs::GeoPose &from,
                             geographic_msgs::GeoPose &to)
  {
    convert(from, to);
  }

  /** @return true if no altitude specified. */
  static inline bool is2D(const geographic_msgs::GeoPoint &pt)
  {
    return (pt.altitude != pt.altitude);
  }

  /** @return true if pose has no altitude. */
  static inline bool is2D(const geographic_msgs::GeoPose &pose)
  {
    return is2D(pose.position);
  }

  /** @return true if point is valid. */
  static inline bool isValid(const geographic_msgs::GeoPoint &pt)
  {
    if (pt.latitude < -90.0 || pt.latitude > 90.0)
      return false;

    if (pt.longitude < -180.0 || pt.longitude >= 180.0)
      return false;

    return true;
  }

  /** @return true if pose is valid. */
  static inline bool isValid(const geographic_msgs::GeoPose &pose)
  {
    // check that orientation quaternion is normalized
    double len2 = (pose.orientation.x * pose.orientation.x
                   + pose.orientation.y * pose.orientation.y
                   + pose.orientation.z * pose.orientation.z
                   + pose.orientation.w * pose.orientation.w);
    if (fabs(len2 - 1.0) > tf::QUATERNION_TOLERANCE)
      return false;

    return isValid(pose.position);
  }

  /** Normalize a WGS 84 geodetic point.
   *
   *  @param pt point to normalize.
   *
   *  Normalizes the longitude to [-180, 180).
   *  Clamps latitude to [-90, 90].
   */
  static inline void normalize(geographic_msgs::GeoPoint &pt)
  {
    pt.longitude =
      (fmod(fmod((pt.longitude + 180.0), 360.0) + 360.0, 360.0) - 180.0);
    pt.latitude = std::min(std::max(pt.latitude, -90.0), 90.0);
  }

  /** @return a 2D WGS 84 geodetic point message. */
  static inline geographic_msgs::GeoPoint toMsg(double lat, double lon)
  {
    geographic_msgs::GeoPoint pt;
    pt.latitude = lat;
    pt.longitude = lon;
    pt.altitude = std::numeric_limits<double>::quiet_NaN();
    return pt;
  }

  /** @return a 3D WGS 84 geodetic point message. */
  static inline geographic_msgs::GeoPoint
    toMsg(double lat, double lon, double alt)
  {
    geographic_msgs::GeoPoint pt;
    pt.latitude = lat;
    pt.longitude = lon;
    pt.altitude = alt;
    return pt;
  }

  /** @return a WGS 84 geodetic point message from a NavSatFix. */
  static inline geographic_msgs::GeoPoint
    toMsg(const sensor_msgs::NavSatFix &fix)
  {
    geographic_msgs::GeoPoint pt;
    pt.latitude = fix.latitude;
    pt.longitude = fix.longitude;
    pt.altitude = fix.altitude;
    return pt;
  }

  /** @return a WGS 84 geodetic point message from another. */
  static inline geographic_msgs::GeoPoint
    toMsg(const geographic_msgs::GeoPoint &from)
  {
    return from;
  }

  /** @return a WGS 84 geodetic pose message from a point and a
   *          quaternion.
   */
  static inline geographic_msgs::GeoPose
    toMsg(const geographic_msgs::GeoPoint &pt,
          const geometry_msgs::Quaternion &q)
  {
    geographic_msgs::GeoPose pose;
    pose.position = pt;
    pose.orientation = q;
    return pose;
  }

  /** @return a WGS 84 geodetic pose message from a NavSatFix and a
   *          quaternion.
   */
  static inline geographic_msgs::GeoPose
    toMsg(const sensor_msgs::NavSatFix &fix,
          const geometry_msgs::Quaternion &q)
  {
    geographic_msgs::GeoPose pose;
    pose.position = toMsg(fix);
    pose.orientation = q;
    return pose;
  }

  /** @return a WGS 84 geodetic pose message from another. */
  static inline geographic_msgs::GeoPose
    toMsg(const geographic_msgs::GeoPose &from)
  {
    return from;
  }

  template <class From, class To>
  void convert(const From &from, To &to)
  {
    fromMsg(toMsg(from), to);
  }

  template <class Same>
  void convert(const Same &from, Same &to)
  {
    if (&from != &to)
      to = from;
  }

}; // namespace geodesy

#endif // _WGS84_H_
