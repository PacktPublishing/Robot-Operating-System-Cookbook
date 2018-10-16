/* -*- mode: C++ -*- */
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2012 Jack O'Quin
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

#ifndef _UNIQUE_ID_H_
#define _UNIQUE_ID_H_ 1

/** @file

    @brief Helper functions for universally unique identifiers and messages.

    @author Jack O'Quin
 */

#include <string>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <ros/ros.h>
#include <uuid_msgs/UniqueID.h>

#include <unique_id/impl/unique_id.h> // private implementation details

/** @brief C++ namespace for unique_id helper functions.
 *
 *  Various ROS components use universally unique identifiers. This
 *  header provides functions for working with a common
 *  uuid_msgs/UniqueID message, and the boost uuid class.
 *
 *   - http://en.wikipedia.org/wiki/Uuid
 *   - http://tools.ietf.org/html/rfc4122.html
 *   - http://www.boost.org/doc/libs/1_42_0/libs/uuid/uuid.html
 *
 *  Programmers are free to create UUID objects using any approved RFC
 *  4122 method. The boost uuid interface supports them all.
 *
 *  Functions in this namespace provide simple APIs, not requiring
 *  detailed knowledge of RFC 4122 or the boost uuid interface.  ROS
 *  applications are likely to need either a random or a name-based
 *  UUID.
 *
 *   - fromRandom() generates a random UUID.
 *   - fromURL() generates a name-based UUID from a URL string.
 */
namespace unique_id
{

/** @brief Create UUID object from UniqueID message.
 *
 *  @param msg uuid_msgs/UniqueID message.
 *  @returns boost::uuids::uuid object.
 */
static inline boost::uuids::uuid fromMsg(uuid_msgs::UniqueID const &msg)
{
  boost::uuids::uuid uu;
  std::copy(msg.uuid.begin(), msg.uuid.end(), uu.begin());
  return uu;
}

/** @brief Generate a random UUID object.
 *
 *  @returns type 4 boost::uuids::uuid object.
 *
 *  Different calls to this function at any time or place will almost
 *  certainly generate different UUIDs. The method used is RFC 4122
 *  variant 4.
 */
static inline boost::uuids::uuid fromRandom(void)
{
  return impl::genRandom();
}

/** @brief Generate UUID from canonical hex string.
 *
 *  @param str string containing canonical hex representation
 *  @returns corresponding boost::uuids::uuid object.
 *
 *  @note This is not a general service for generating a UUID from an
 *  arbitrary character string. The fromURL() function will do that
 *  for any Uniform Resource Identifier.
 *
 *  The canonical hex string is a human-readable representation of a
 *  correctly-formatted sixteen-byte UUID. In addition to the dashes,
 *  it should contain exactly 32 hexadecimal digits.  The @a str can
 *  be any accepted by the boost uuid string generator, but that is
 *  not well-defined. The format produced by toHexString() works
 *  reliably: "01234567-89ab-cdef-0123-456789abcdef".
 *
 *  @warning Strings not accepted by boost may produce undefined
 *  results: perhaps throwing a @c std::runtime_error exception, or
 *  silently ignoring parts of the string.
 */
static inline boost::uuids::uuid fromHexString(std::string const &str)
{
  return impl::genString(str);
}

/** @brief Generate UUID from Uniform Resource Identifier.
 *
 *  @param url URL for identifier creation.
 *  @returns type 5 boost::uuids::uuid object.
 *
 *  Matching @a url strings must yield the same UUID. Different @a url
 *  strings will almost certainly generate different UUIDs. The method
 *  used is RFC 4122 variant 5, computing the SHA-1 hash of the @a
 *  url.
 *
 *  For any given @a url, this function returns the same UUID as the
 *  corresponding Python @c unique_id.fromURL() function.
 *
 *  For example, Open Street Map identifiers are encoded like this,
 *  with decimal representations of the integer OSM node, way, or
 *  relation identifiers appended to the URL:
 *
 *   - fromURL("http://openstreetmap.org/node/123456789")
 *   - fromURL("http://openstreetmap.org/way/6543210")
 *   - fromURL("http://openstreetmap.org/relation/999999")
 */
static inline boost::uuids::uuid fromURL(std::string const &url)
{
  return impl::genURL(url);
}

/** @brief Generate a Time Based UUID object. Users are recommended to seed the random
 *         number generator using srand from the calling application to generate the clock_id
 *
 *  @param timestamp The ros::Time timestamp for UUID generation
 *  @param hw_addr A 48-bit (6 octets) network address assigned to the 48 LSBs
 *  @returns type 1 boost::uuids::uuid object.
 *
 *  Different calls to this function at any time or place will almost
 *  certainly generate different UUIDs. The method used is RFC 4122
 *  version 1.
 */
static inline boost::uuids::uuid fromTime(ros::Time timestamp, uint64_t hw_addr)
{
  return impl::genTime(timestamp, hw_addr);
}

/** @brief Create a UniqueID message from a UUID object.
 *
 *  @param uu boost::uuids::uuid object.
 *  @returns uuid_msgs/UniqueID message.
 */
static inline uuid_msgs::UniqueID toMsg(boost::uuids::uuid const &uu)
{
  uuid_msgs::UniqueID msg;
  std::copy(uu.begin(), uu.end(), msg.uuid.begin());
  return msg;
}

/** @brief Get the canonical string representation for a boost UUID.
 *
 *  @param uu boost::uuids::uuid object.
 *  @returns canonical UUID hex string: "01234567-89ab-cdef-0123-456789abcdef".
 *
 *  A @c boost::uuids::uuid object yields the same representation via
 *  its @c << operator or @c to_string() function.
 */
static inline std::string toHexString(boost::uuids::uuid const &uu)
{
  return boost::uuids::to_string(uu);
}

/** @brief Get the canonical string representation for a UniqueID message.
 *
 *  @param msg uuid_msgs/UniqueID message.
 *  @returns canonical UUID hex string: "01234567-89ab-cdef-0123-456789abcdef".
 */
static inline std::string toHexString(uuid_msgs::UniqueID const &msg)
{
  return boost::uuids::to_string(fromMsg(msg));
}

} // end namespace unique_id

#endif // _UNIQUE_ID_H_
