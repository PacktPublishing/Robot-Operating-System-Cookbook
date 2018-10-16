/* $Id: c7a178b0bbba69e8bf70b12ea1fe76e0cc5a4bff $ */

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011 Jack O'Quin
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

#include <gtest/gtest.h>
#include <limits>
#include "geodesy/wgs84.h"

///////////////////////////////////////////////////////////////
// Utility functions
///////////////////////////////////////////////////////////////

// check that normalize of (lat1, lon1) yields (lat2, lon2)
void check_normalize(double lat1, double lon1, double lat2, double lon2)
{
 double epsilon = 1e-9;
 geographic_msgs::GeoPoint pt;
 pt.latitude = lat1;
 pt.longitude = lon1;
 geodesy::normalize(pt);
 EXPECT_NEAR(lat2, pt.latitude, epsilon);
 EXPECT_NEAR(lon2, pt.longitude, epsilon);
}

///////////////////////////////////////////////////////////////
// Test cases
///////////////////////////////////////////////////////////////

// Test null point constructor
TEST(GeoPoint, nullConstructor)
{
  geographic_msgs::GeoPoint pt;

  EXPECT_FALSE(geodesy::is2D(pt));
  EXPECT_TRUE(geodesy::isValid(pt));
}

// Test point with no altitude
TEST(GeoPoint, noAltitude)
{
  geographic_msgs::GeoPoint pt(geodesy::toMsg(0.0, 0.0));
  EXPECT_TRUE(geodesy::is2D(pt));
}

// Test creating point from NavSatFix message
TEST(GeoPoint, fromNavSatFix)
{
  double lat = 30.0;
  double lon = -97.0;
  double alt = 208.0;
  sensor_msgs::NavSatFix fix;
  fix.latitude = lat;
  fix.longitude = lon;
  fix.altitude = alt;

  geographic_msgs::GeoPoint pt(geodesy::toMsg(fix));

  EXPECT_FALSE(geodesy::is2D(pt));
  EXPECT_TRUE(geodesy::isValid(pt));

  EXPECT_EQ(pt.latitude, lat);
  EXPECT_EQ(pt.longitude, lon);
  EXPECT_EQ(pt.altitude, alt);
}

// Test point with valid altitude
TEST(GeoPoint, hasAltitude)
{
  geographic_msgs::GeoPoint pt(geodesy::toMsg(30.0, 206.0, -97.0));
  EXPECT_FALSE(geodesy::is2D(pt));

  pt.altitude = -100.0;
  EXPECT_FALSE(geodesy::is2D(pt));

  pt.altitude = 20000.0;
  EXPECT_FALSE(geodesy::is2D(pt));

  pt.altitude = 0.0;
  EXPECT_FALSE(geodesy::is2D(pt));
}

// Test valid latitudes and longitudes
TEST(GeoPoint, validLatLong)
{
  geographic_msgs::GeoPoint pt;

  pt.latitude = 90.0;
  EXPECT_TRUE(geodesy::isValid(pt));

  pt.latitude = -90.0;
  EXPECT_TRUE(geodesy::isValid(pt));

  pt.latitude = 30.0;
  pt.longitude = -97.0;
  EXPECT_TRUE(geodesy::isValid(pt));

  pt.longitude = 179.999999;
  EXPECT_TRUE(geodesy::isValid(pt));

  pt.longitude = -180.0;
  EXPECT_TRUE(geodesy::isValid(pt));
}

// Test valid latitudes and longitudes
TEST(GeoPoint, invalidLatLong)
{
  geographic_msgs::GeoPoint pt;

  pt.latitude = 90.001;
  EXPECT_FALSE(geodesy::isValid(pt));

  pt.latitude = -90.3;
  EXPECT_FALSE(geodesy::isValid(pt));

  pt.latitude = 30.0;
  pt.longitude = -1000.0;
  EXPECT_FALSE(geodesy::isValid(pt));

  pt.longitude = 180.0;
  EXPECT_FALSE(geodesy::isValid(pt));

  pt.longitude = 180.0001;
  EXPECT_FALSE(geodesy::isValid(pt));

  pt.longitude = -180.999;
  EXPECT_FALSE(geodesy::isValid(pt));
}

TEST(GeoPoint, normalize)
{
 check_normalize(0, 0, 0, 0);

 // longitude range
 check_normalize(0, 90,  0, 90);
 check_normalize(0, 180, 0, -180);
 check_normalize(0, 270, 0, -90);
 check_normalize(0, 360, 0, 0);
 check_normalize(0, 450, 0, 90);
 check_normalize(0, 540, 0, -180);
 check_normalize(0, 630, 0, -90);
 check_normalize(0, 720, 0, 0);

 check_normalize(0, -90,  0, -90);
 check_normalize(0, -180, 0, -180);
 check_normalize(0, -270, 0, 90);
 check_normalize(0, -360, 0, 0);
 check_normalize(0, -450, 0, -90);
 check_normalize(0, -540, 0, -180);
 check_normalize(0, -630, 0, 90);
 check_normalize(0, -720, 0, 0);

 check_normalize(0, 45,   0, 45);
 check_normalize(0, 135,  0, 135);
 check_normalize(0, 225,  0, -135);
 check_normalize(0, 315,  0, -45);

 check_normalize(0, -45,   0, -45);
 check_normalize(0, -135,  0, -135);
 check_normalize(0, -225,  0, 135);
 check_normalize(0, -315,  0, 45);

 check_normalize(0, 91,  0, 91);
 check_normalize(0, 181, 0, -179);
 check_normalize(0, 271, 0, -89);
 check_normalize(0, 361, 0, 1);
 check_normalize(0, 451, 0, 91);

 check_normalize(0, -91,  0, -91);
 check_normalize(0, -181, 0, 179);
 check_normalize(0, -271, 0, 89);
 check_normalize(0, -361, 0, -1);
 check_normalize(0, -451, 0, -91);

 // latitude range
 check_normalize(15, 0, 15, 0);
 check_normalize(30, 0, 30, 0);
 check_normalize(45, 0, 45, 0);
 check_normalize(60, 0, 60, 0);
 check_normalize(75, 0, 75, 0);
 check_normalize(90, 0, 90, 0);
 check_normalize(105, 0, 90, 0);
 check_normalize(89.999999, 0, 89.999999, 0);
 check_normalize(90.000001, 0, 90, 0);

 check_normalize(-15, 0, -15, 0);
 check_normalize(-30, 0, -30, 0);
 check_normalize(-45, 0, -45, 0);
 check_normalize(-60, 0, -60, 0);
 check_normalize(-75, 0, -75, 0);
 check_normalize(-90, 0, -90, 0);
 check_normalize(-105, 0, -90, 0);
 check_normalize(-89.999999, 0, -89.999999, 0);
 check_normalize(-90.000001, 0, -90, 0);

}

// Test null pose constructor
TEST(GeoPose, nullConstructor)
{
  geographic_msgs::GeoPose pose;

  EXPECT_FALSE(geodesy::is2D(pose));
  EXPECT_FALSE(geodesy::isValid(pose));
}

// Test validity of various quaternions
TEST(GeoPose, quaternionValidity)
{
  geographic_msgs::GeoPose pose;
  EXPECT_FALSE(geodesy::isValid(pose));

  pose.orientation.x = 1.0;             // identity quaternion
  EXPECT_TRUE(geodesy::isValid(pose));

  pose.orientation.x = 0.7071;          // also valid
  pose.orientation.y = 0.7071;
  EXPECT_TRUE(geodesy::isValid(pose));

  pose.orientation.x = 0.8071;          // not normalized
  pose.orientation.y = 0.8071;
  EXPECT_FALSE(geodesy::isValid(pose));
}

// Test trivial point conversion
TEST(Convert, GeoPointToGeoPoint)
{
  geographic_msgs::GeoPoint pt1(geodesy::toMsg(30.0, 206.0, -97.0));
  geographic_msgs::GeoPoint pt2;

  geodesy::convert(pt1, pt2);

  EXPECT_EQ(pt1.latitude, pt2.latitude);
  EXPECT_EQ(pt1.longitude, pt2.longitude);
  EXPECT_EQ(pt1.altitude, pt2.altitude);
}

// Test trivial pose conversion
TEST(Convert, GeoPoseToGeoPose)
{
  geographic_msgs::GeoPoint pt(geodesy::toMsg(30.0, 206.0, -97.0));
  geometry_msgs::Quaternion q;
  geographic_msgs::GeoPose pose1(geodesy::toMsg(pt, q));
  geographic_msgs::GeoPose pose2;

  geodesy::convert(pose1, pose2);

  EXPECT_EQ(pose1.position.latitude, pose2.position.latitude);
  EXPECT_EQ(pose1.position.longitude, pose2.position.longitude);
  EXPECT_EQ(pose1.position.altitude, pose2.position.altitude);
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  // run the tests in this thread
  return RUN_ALL_TESTS();
}
