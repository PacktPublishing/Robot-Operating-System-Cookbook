/* $Id: 8b94f2e1fa9389bb9b9229465157c45e84029c98 $ */

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2011 Chuck Gantz, Jack O'Quin
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

//#include <exception>
#include <ros/ros.h>
#include <angles/angles.h>
#include <geodesy/utm.h>

/**  @file
   
     @brief Universal Transverse Mercator transforms.

     Functions to convert WGS 84 (ellipsoidal) latitude and longitude
     to and from (Euclidean) UTM coordinates.

     @todo handle Universal Polar Stereographic (UPS) zones

     @author Chuck Gantz, Globalstar, Inc.
     @author Jack O'Quin, for ROS interface
*/

namespace geodesy
{

// WGS84 Parameters
#define WGS84_A		6378137.0		// major axis
#define WGS84_B		6356752.31424518	// minor axis
#define WGS84_F		0.0033528107		// ellipsoid flattening
#define WGS84_E		0.0818191908		// first eccentricity
#define WGS84_EP	0.0820944379		// second eccentricity

// UTM Parameters
#define UTM_K0		0.9996			// scale factor
#define UTM_FE		500000.0		// false easting
#define UTM_FN_N	0.0           // false northing, northern hemisphere
#define UTM_FN_S	10000000.0    // false northing, southern hemisphere
#define UTM_E2		(WGS84_E*WGS84_E)	// e^2
#define UTM_E4		(UTM_E2*UTM_E2)		// e^4
#define UTM_E6		(UTM_E4*UTM_E2)		// e^6
#define UTM_EP2		(UTM_E2/(1-UTM_E2))	// e'^2


/**
 * Determine the correct UTM band letter for the given latitude.
 * (Does not currently handle polar (UPS) zones: A, B, Y, Z).
 *
 * @return ' ' if latitude is outside the UTM limits of +84 to -80
 */
static char UTMBand(double Lat, double Lon)
{
  char LetterDesignator;

  if     ((84 >= Lat) && (Lat >= 72))  LetterDesignator = 'X';
  else if ((72 > Lat) && (Lat >= 64))  LetterDesignator = 'W';
  else if ((64 > Lat) && (Lat >= 56))  LetterDesignator = 'V';
  else if ((56 > Lat) && (Lat >= 48))  LetterDesignator = 'U';
  else if ((48 > Lat) && (Lat >= 40))  LetterDesignator = 'T';
  else if ((40 > Lat) && (Lat >= 32))  LetterDesignator = 'S';
  else if ((32 > Lat) && (Lat >= 24))  LetterDesignator = 'R';
  else if ((24 > Lat) && (Lat >= 16))  LetterDesignator = 'Q';
  else if ((16 > Lat) && (Lat >= 8))   LetterDesignator = 'P';
  else if (( 8 > Lat) && (Lat >= 0))   LetterDesignator = 'N';
  else if (( 0 > Lat) && (Lat >= -8))  LetterDesignator = 'M';
  else if ((-8 > Lat) && (Lat >= -16)) LetterDesignator = 'L';
  else if((-16 > Lat) && (Lat >= -24)) LetterDesignator = 'K';
  else if((-24 > Lat) && (Lat >= -32)) LetterDesignator = 'J';
  else if((-32 > Lat) && (Lat >= -40)) LetterDesignator = 'H';
  else if((-40 > Lat) && (Lat >= -48)) LetterDesignator = 'G';
  else if((-48 > Lat) && (Lat >= -56)) LetterDesignator = 'F';
  else if((-56 > Lat) && (Lat >= -64)) LetterDesignator = 'E';
  else if((-64 > Lat) && (Lat >= -72)) LetterDesignator = 'D';
  else if((-72 > Lat) && (Lat >= -80)) LetterDesignator = 'C';
  // '_' is an error flag, the Latitude is outside the UTM limits
  else LetterDesignator = ' ';

  return LetterDesignator;
}

/** Convert UTM point to WGS 84 geodetic point.
 *
 *  Equations from USGS Bulletin 1532
 *
 *  @param from WGS 84 point message.
 *  @return UTM point.
 */
geographic_msgs::GeoPoint toMsg(const UTMPoint &from)
{
  //remove 500,000 meter offset for longitude
  double x = from.easting - 500000.0;
  double y = from.northing;

  double k0 = UTM_K0;
  double a = WGS84_A;
  double eccSquared = UTM_E2;
  double eccPrimeSquared;
  double e1 = (1-sqrt(1-eccSquared))/(1+sqrt(1-eccSquared));
  double N1, T1, C1, R1, D, M;
  double LongOrigin;
  double mu, phi1Rad;

  if ((from.band - 'N') < 0)
    {
      //point is in southern hemisphere
      //remove 10,000,000 meter offset used for southern hemisphere
      y -= 10000000.0;
    }

  //+3 puts origin in middle of zone
  LongOrigin = (from.zone - 1)*6 - 180 + 3;
  eccPrimeSquared = (eccSquared)/(1-eccSquared);

  M = y / k0;
  mu = M/(a*(1-eccSquared/4-3*eccSquared*eccSquared/64
             -5*eccSquared*eccSquared*eccSquared/256));

  phi1Rad = mu + ((3*e1/2-27*e1*e1*e1/32)*sin(2*mu) 
                  + (21*e1*e1/16-55*e1*e1*e1*e1/32)*sin(4*mu)
                  + (151*e1*e1*e1/96)*sin(6*mu));

  N1 = a/sqrt(1-eccSquared*sin(phi1Rad)*sin(phi1Rad));
  T1 = tan(phi1Rad)*tan(phi1Rad);
  C1 = eccPrimeSquared*cos(phi1Rad)*cos(phi1Rad);
  R1 = a*(1-eccSquared)/pow(1-eccSquared*sin(phi1Rad)*sin(phi1Rad), 1.5);
  D = x/(N1*k0);

  // function result
  geographic_msgs::GeoPoint to;
  to.altitude = from.altitude;
  to.latitude =
    phi1Rad - ((N1*tan(phi1Rad)/R1)
               *(D*D/2
                 -(5+3*T1+10*C1-4*C1*C1-9*eccPrimeSquared)*D*D*D*D/24
                 +(61+90*T1+298*C1+45*T1*T1-252*eccPrimeSquared
                   -3*C1*C1)*D*D*D*D*D*D/720));
  to.latitude = angles::to_degrees(to.latitude);
  to.longitude = ((D-(1+2*T1+C1)*D*D*D/6
                   +(5-2*C1+28*T1-3*C1*C1+8*eccPrimeSquared+24*T1*T1)
                   *D*D*D*D*D/120)
                  / cos(phi1Rad));
  to.longitude = LongOrigin + angles::to_degrees(to.longitude);

  // Normalize latitude and longitude to valid ranges.
  normalize(to);
  return to;
}

/** Convert WGS 84 geodetic point to UTM point.
 *
 *  Equations from USGS Bulletin 1532
 *
 *  @param from WGS 84 point message.
 *  @param to UTM point.
 */
void fromMsg(const geographic_msgs::GeoPoint &from, UTMPoint &to)
{
  double Lat = from.latitude;
  double Long = from.longitude;

  double a = WGS84_A;
  double eccSquared = UTM_E2;
  double k0 = UTM_K0;

  double LongOrigin;
  double eccPrimeSquared;
  double N, T, C, A, M;
	
  // Make sure the longitude is between -180.00 .. 179.9
  // (JOQ: this is broken for Long < -180, do a real normalize)
  double LongTemp = (Long+180)-int((Long+180)/360)*360-180;
  double LatRad = angles::from_degrees(Lat);
  double LongRad = angles::from_degrees(LongTemp);
  double LongOriginRad;

  to.altitude = from.altitude;
  to.zone = int((LongTemp + 180)/6) + 1;
  
  if( Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0 )
    to.zone = 32;

  // Special zones for Svalbard
  if( Lat >= 72.0 && Lat < 84.0 ) 
    {
      if(      LongTemp >= 0.0  && LongTemp <  9.0 ) to.zone = 31;
      else if( LongTemp >= 9.0  && LongTemp < 21.0 ) to.zone = 33;
      else if( LongTemp >= 21.0 && LongTemp < 33.0 ) to.zone = 35;
      else if( LongTemp >= 33.0 && LongTemp < 42.0 ) to.zone = 37;
    }
  // +3 puts origin in middle of zone
  LongOrigin = (to.zone - 1)*6 - 180 + 3; 
  LongOriginRad = angles::from_degrees(LongOrigin);

  // compute the UTM band from the latitude
  to.band = UTMBand(Lat, LongTemp);
#if 0
  if (to.band == ' ')
    throw std::range_error;
#endif

  eccPrimeSquared = (eccSquared)/(1-eccSquared);

  N = a/sqrt(1-eccSquared*sin(LatRad)*sin(LatRad));
  T = tan(LatRad)*tan(LatRad);
  C = eccPrimeSquared*cos(LatRad)*cos(LatRad);
  A = cos(LatRad)*(LongRad-LongOriginRad);

  M = a*((1 - eccSquared/4 - 3*eccSquared*eccSquared/64
          - 5*eccSquared*eccSquared*eccSquared/256) * LatRad 
         - (3*eccSquared/8 + 3*eccSquared*eccSquared/32
            + 45*eccSquared*eccSquared*eccSquared/1024)*sin(2*LatRad)
         + (15*eccSquared*eccSquared/256
            + 45*eccSquared*eccSquared*eccSquared/1024)*sin(4*LatRad) 
         - (35*eccSquared*eccSquared*eccSquared/3072)*sin(6*LatRad));
	
  to.easting = (double)
    (k0*N*(A+(1-T+C)*A*A*A/6
           + (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120)
     + 500000.0);

  to.northing = (double)
    (k0*(M+N*tan(LatRad)
         *(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
           + (61-58*T+T*T+600*C-330*eccPrimeSquared)*A*A*A*A*A*A/720)));

  if(Lat < 0)
    {
      //10000000 meter offset for southern hemisphere
      to.northing += 10000000.0;
    }
}

/** @return true if point is valid. */
bool isValid(const UTMPoint &pt)
{
  if (pt.zone < 1 || pt.zone > 60)
    return false;

  if (!isupper(pt.band) || pt.band == 'I' || pt.band == 'O')
    return false;

  // The Universal Polar Stereographic bands are not currently
  // supported.  When they are: A, B, Y and Z will be valid (and the
  // zone number will be ignored).
  if (pt.band < 'C' || pt.band > 'X')
    return false;

  return true;
}
 
/** Create UTM point from WGS 84 geodetic point. */
UTMPoint::UTMPoint(const geographic_msgs::GeoPoint &pt)
{
  fromMsg(pt, *this);
}

/** Convert WGS 84 geodetic pose to UTM pose.
 *
 *  @param from WGS 84 pose message.
 *  @param to UTM pose.
 *
 *  @todo define the orientation transformation properly
 */
void fromMsg(const geographic_msgs::GeoPose &from, UTMPose &to)
{
  fromMsg(from.position, to.position);
  to.orientation = from.orientation;
}

/** @return true if pose is valid. */
bool isValid(const UTMPose &pose)
{
  if (!isValid(pose.position))
    return false;

  // check that orientation quaternion is normalized
  double len2 = (pose.orientation.x * pose.orientation.x
                 + pose.orientation.y * pose.orientation.y
                 + pose.orientation.z * pose.orientation.z
                 + pose.orientation.w * pose.orientation.w);
  return fabs(len2 - 1.0) <= tf::QUATERNION_TOLERANCE;
}

} // end namespace geodesy
