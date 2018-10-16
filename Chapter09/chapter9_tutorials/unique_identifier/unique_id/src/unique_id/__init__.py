# Software License Agreement (BSD License)
#
# Copyright (C) 2012, Jack O'Quin
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the author nor of other contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Python module for unique_id helper functions.

Various ROS components use universally unique identifiers
(UUID_). This module provides functions for working with a common
`uuid_msgs/UniqueID`_ message, and the standard Python
:class:`uuid.UUID` class.

Programmers are free to create UUID objects using any approved `RFC
4122`_ method. The standard Python :py:mod:`uuid` module supports them
all.

Functions in this module provide simple APIs, not requiring detailed
knowledge of `RFC 4122`_ or the :py:mod:`uuid` interface.  ROS
applications are likely to need either a random or a name-based UUID.

 * :func:`fromRandom` generates a random UUID.
 * :func:`fromURL` generates a name-based UUID from a URL string.

.. _`uuid_msgs/UniqueID`: http://ros.org/doc/api/uuid_msgs/html/msg/UniqueID.html
.. _`RFC 4122`: http://tools.ietf.org/html/rfc4122.html
.. _UUID: http://en.wikipedia.org/wiki/Uuid

"""

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

from uuid_msgs.msg import UniqueID

import uuid

def fromMsg(msg):
    """Create UUID object from UniqueID message.

    :param msg: `uuid_msgs/UniqueID`_ message.
    :returns: :class:`uuid.UUID` object.
    """
    return uuid.UUID(bytes = msg.uuid)

def fromRandom():
    """Generate a random UUID object.

    :returns: type 4 :class:`uuid.UUID` object.

    Different calls to this function at any time or place will almost
    certainly generate different UUIDs. The method used is `RFC 4122`_
    variant 4.
    """
    return uuid.uuid4()

def fromURL(url):
    """Generate UUID from Uniform Resource Locator.

    :param url: URL for identifier creation.
    :returns: type 5 :class:`uuid.UUID` object.

    Matching *url* strings must yield the same UUID. Different *url*
    strings will almost certainly generate different UUIDs. The method
    used is `RFC 4122`_ variant 5, computing the SHA-1 hash of the
    *url*.

    For any given *url*, this function returns the same UUID as the
    corresponding C++ `unique_id::fromURL()` function.

    For example, Open Street Map identifiers are encoded like this,
    with decimal representations of the integer OSM node, way, or
    relation identifiers appended to the URL::

        fromURL('http://openstreetmap.org/node/' + str(node_id))
        fromURL('http://openstreetmap.org/way/' + str(way_id))
        fromURL('http://openstreetmap.org/relation/' + str(rel_id))

    """
    return uuid.uuid5(uuid.NAMESPACE_URL, url)

def fromTime(timestamp, hw_addr):
    """Generate a Time Based UUID object.

    :param timestamp: The rospy.Time timestamp for UUID generation
    :param hw_addr: A 48-bit long representing the network address
    :returns: type 1 :class:`uuid.UUID` object

    Different calls to this function at any time or place will almost
    certainly generate different UUIDs. The method used is RFC 4122
    version 1.
    """
    uu = uuid.uuid1(hw_addr)
    offset = 122192928000000000
    nano_epoch = long(timestamp.secs / (100 * 1e-9) + timestamp.nsecs / 100)
    # return nano_epoch
    nano_rfc = nano_epoch + offset
    data = (nano_rfc & 0x0000000FFFFFFFF, (nano_rfc >> 32) & 0x000FFFF, (nano_rfc >> 48) | 0x1000,
            uu.fields[3], uu.fields[4], uu.fields[5])
    return uuid.UUID(fields=data)

def toMsg(uuid_obj):
    """Create a UniqueID message from a UUID object.

    :param uuid_obj: standard Python :class:`uuid.UUID` object.
    :returns: `uuid_msgs/UniqueID`_ message.
    """
    return UniqueID(uuid = uuid_obj.bytes)

def toHexString(msg):
    """Get the canonical hexadecimal string representation for a UniqueID message.

    :param msg: `uuid_msgs/UniqueID`_ message.
    :returns: UUID hex string: '01234567-89ab-cdef-0123-456789abcdef'.

    A :class:`uuid.UUID` object yields the same representation via the
    :py:func:`str` function.
    """
    return str(fromMsg(msg))
