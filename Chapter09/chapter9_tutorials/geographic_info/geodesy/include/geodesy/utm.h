/* -*- mode: C++ -*- */
/* $Id: 1f3d19770631d66c82ea08b4324abcea523ef92b $ */

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

#ifndef _UTM_H_
#define _UTM_H_

#include <limits>
#include <ctype.h>
#include <iostream>
#include <iomanip>

#include <tf/tf.h>

#include <geodesy/wgs84.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

/** @file

    @brief Universal Transverse Mercator coordinates

    For outdoor robotics applications, Euclidean projections like UTM
    are easier to work with than latitude and longitude.  This system
    is slightly more general than strict UTM.  It is based on the
    Military Grid Reference System (MGRS), which can be extended to
    cover the poles, allowing well-defined transformations for every
    latitude and longitude.

    @todo add Universal Polar Stereographic support

    @author Jack O'Quin
 */

namespace geodesy
{

/** Universal Transverse Mercator (UTM) point.
 *
 *  The @c altitude may be specified (3D) or not (2D).  The @c
 *  altitude of a 2D point is not a number (NaN).
 *
 *  Including the top-level grid zone designator (GZD) from the
 *  Military Grid Reference System (MGRS) permits unambiguous use of
 *  Universal Polar Stereographic (UPS) coordinates for the polar
 *  regions not covered by UTM, making this representation more
 *  general than pure UTM.
 */
class UTMPoint
{
 public:

  /** Null constructor. Makes a 2D, invalid point object. */
  UTMPoint():
    easting(0.0),
    northing(0.0),
    altitude(std::numeric_limits<double>::quiet_NaN()),
    zone(0),
    band(' ')
  {}

  /** Copy constructor. */
  UTMPoint(const UTMPoint &that):
    easting(that.easting),
    northing(that.northing),
    altitude(that.altitude),
    zone(that.zone),
    band(that.band)
  {}
  
  UTMPoint(const geographic_msgs::GeoPoint &pt);

  /** Create a flattened 2-D grid point. */
  UTMPoint(double _easting, double _northing, uint8_t _zone, char _band):
    easting(_easting),
    northing(_northing),
    altitude(std::numeric_limits<double>::quiet_NaN()),
    zone(_zone),
    band(_band)
  {}

  /** Create a 3-D grid point. */
  UTMPoint(double _easting, double _northing, double _altitude,
            uint8_t _zone, char _band):
    easting(_easting),
    northing(_northing),
    altitude(_altitude),
    zone(_zone),
    band(_band)
  {}

  // data members
  double easting;           ///< easting within grid zone [meters]
  double northing;          ///< northing within grid zone [meters] 
  double altitude;          ///< altitude above ellipsoid [meters] or NaN
  uint8_t zone;             ///< UTM longitude zone number
  char   band;              ///< MGRS latitude band letter

}; // class UTMPoint

/** Universal Transverse Mercator (UTM) pose */
class UTMPose
{
 public:

  /** Null constructor. Makes a 2D, invalid pose object. */
  UTMPose():
    position(),
    orientation()
  {}

  /** Copy constructor. */
  UTMPose(const UTMPose &that):
    position(that.position),
    orientation(that.orientation)
  {}
  
  /** Create from a WGS 84 geodetic pose. */
  UTMPose(const geographic_msgs::GeoPose &pose):
    position(pose.position),
    orientation(pose.orientation)
  {}

  /** Create from a UTMPoint and a quaternion. */
  UTMPose(UTMPoint pt,
          const geometry_msgs::Quaternion &q):
    position(pt),
    orientation(q)
  {}

  /** Create from a WGS 84 geodetic point and a quaternion. */
  UTMPose(const geographic_msgs::GeoPoint &pt,
          const geometry_msgs::Quaternion &q):
    position(pt),
    orientation(q)
  {}

  // data members
  UTMPoint position;
  geometry_msgs::Quaternion orientation;

}; // class UTMPose

// conversion function prototypes
void fromMsg(const geographic_msgs::GeoPoint &from, UTMPoint &to);
void fromMsg(const geographic_msgs::GeoPose &from, UTMPose &to);
geographic_msgs::GeoPoint toMsg(const UTMPoint &from);
geographic_msgs::GeoPose toMsg(const UTMPose &from);

/** @return true if no altitude specified. */
static inline bool is2D(const UTMPoint &pt)
{
  // true if altitude is a NaN
  return (pt.altitude != pt.altitude);
}

/** @return true if no altitude specified. */
static inline bool is2D(const UTMPose &pose)
{
  // true if position has no altitude
  return is2D(pose.position);
}

bool isValid(const UTMPoint &pt);
bool isValid(const UTMPose &pose);

/** Normalize UTM point.
 *
 *  Ensures the point is within its canonical grid zone.
 */
static inline void normalize(UTMPoint &pt)
{
  geographic_msgs::GeoPoint ll(toMsg(pt));
  normalize(ll);
  fromMsg(ll, pt);
}

/** Output stream operator for UTM point. */
static inline std::ostream& operator<<(std::ostream& out, const UTMPoint &pt)
{
  out << "(" << std::setprecision(10) << pt.easting << ", "
      << pt.northing << ", " << std::setprecision(6) << pt.altitude
      << " [" << (unsigned) pt.zone << pt.band << "])";
  return out;
}

/** Output stream operator for UTM pose. */
static inline std::ostream& operator<<(std::ostream& out, const UTMPose &pose)
{
  out << pose.position << ", (["
      << pose.orientation.x << ", "
      << pose.orientation.y << ", "
      << pose.orientation.z << "], "
      << pose.orientation.w << ")";
  return out;
}

/** @return true if two points have the same Grid Zone Designator */
static inline bool sameGridZone(const UTMPoint &pt1, const UTMPoint &pt2)
{
  return ((pt1.zone == pt2.zone) && (pt1.band == pt2.band));
}

/** @return true if two poses have the same Grid Zone Designator */
static inline bool sameGridZone(const UTMPose &pose1, const UTMPose &pose2)
{
  return sameGridZone(pose1.position, pose2.position);
}

/** @return a geometry Point corresponding to a UTM point. */
static inline geometry_msgs::Point toGeometry(const UTMPoint &from)
{
  geometry_msgs::Point to;
  to.x = from.easting;
  to.y = from.northing;
  to.z = from.altitude;
  return to;
}

/** @return a geometry Pose corresponding to a UTM pose. */
static inline geometry_msgs::Pose toGeometry(const UTMPose &from)
{
  geometry_msgs::Pose to;
  to.position = toGeometry(from.position);
  to.orientation = from.orientation;
  return to;
}

}; // namespace geodesy

#endif // _UTM_H_
