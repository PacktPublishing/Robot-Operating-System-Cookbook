#!/usr/bin/env python

import unittest

from geographic_msgs.msg import GeographicMap
from geographic_msgs.msg import GeoPoint
from geographic_msgs.msg import WayPoint
from geometry_msgs.msg import Point
from uuid_msgs.msg import UniqueID

from geodesy.wu_point import *

def fromLatLong(lat, lon, alt=float('nan')):
    """Generate WayPoint from latitude, longitude and (optional) altitude.

    :returns: minimal WayPoint object just for test cases.
    """
    geo_pt = GeoPoint(latitude = lat, longitude = lon, altitude = alt)
    return WayPoint(position = geo_pt)

class TestWuPoint(unittest.TestCase):
    """Unit tests for WuPoint classes.
    """

    def test_real_point(self):
        ll = GeoPoint(latitude = 30.385315,
                      longitude = -97.728524,
                      altitude = 209.0)
        msg = WayPoint(position = ll)
        pt = WuPoint(msg)
        self.assertEqual(pt.toWayPoint(), msg)
        self.assertEqual(str(pt.utm),
                         'UTM: [622159.338, 3362168.303, 209.000, 14R]')

        point_xyz = pt.toPoint()
        self.assertAlmostEqual(point_xyz.x, 622159.338, places = 3)
        self.assertAlmostEqual(point_xyz.y, 3362168.303, places = 3)
        self.assertAlmostEqual(point_xyz.z, 209.0, places = 3)

        point_xy = pt.toPointXY()
        self.assertAlmostEqual(point_xy.x, 622159.338, places = 3)
        self.assertAlmostEqual(point_xy.y, 3362168.303, places = 3)
        self.assertAlmostEqual(point_xy.z, 0.0, places = 3)

    def test_valid_points(self):
        lon = -177.0
        zone = 1
        while lon < 180.0:
            pt = WuPoint(fromLatLong(-80.0, lon))
            self.assertEqual(pt.utm.gridZone(), (zone, 'C'))
            pt = WuPoint(fromLatLong(-30.385315, lon))
            self.assertEqual(pt.utm.gridZone(), (zone, 'J'))
            pt = WuPoint(fromLatLong(-0.000001, lon))
            self.assertEqual(pt.utm.gridZone(), (zone, 'M'))
            pt = WuPoint(fromLatLong(0.0, lon))
            self.assertEqual(pt.utm.gridZone(), (zone, 'N'))
            pt = WuPoint(fromLatLong(30.385315, lon))
            self.assertEqual(pt.utm.gridZone(), (zone, 'R'))
            pt = WuPoint(fromLatLong(84.0, lon))
            self.assertEqual(pt.utm.gridZone(), (zone, 'X'))
            lon += 6.0
            zone += 1

    def test_invalid_points(self):
        self.assertRaises(ValueError, WuPoint,
                          fromLatLong(90.385315, -97.728524))
        self.assertRaises(ValueError, WuPoint,
                          fromLatLong(30.385315, -197.728524))
        # this will be valid when we add UPS support for the poles:
        self.assertRaises(ValueError, WuPoint,
                          fromLatLong(-80.385315,-97.728524))

    def test_empty_point_set(self):
        # test WuPointSet iterator with empty list
        wupts = WuPointSet(GeographicMap().points)
        i = 0
        for w in wupts:
            self.fail(msg='there are no points in this map')
            i += 1
        self.assertEqual(i, 0)

        uu = 'da7c242f-2efe-5175-9961-49cc621b80b9'
        self.assertEqual(wupts.get(uu), None)

    def test_three_point_set(self):
        # test data
        uuids = ['da7c242f-2efe-5175-9961-49cc621b80b9',
                 '812f1c08-a34b-5a21-92b9-18b2b0cf4950',
                 '6f0606f6-a776-5940-b5ea-5e889b32c712']
        latitudes = [30.3840168, 30.3857290, 30.3866750]
        longitudes = [-97.7282100, -97.7316754, -97.7270967]
        eastings = [622191.124, 621856.023, 622294.785]
        northings = [3362024.764, 3362210.790, 3362320.569]

        points = []
        for i in range(len(uuids)):
            latlon = GeoPoint(latitude = latitudes[i],
                              longitude = longitudes[i])
            points.append(WayPoint(id = UniqueID(uuid = uuids[i]),
                                   position = latlon))
    
        # test iterator
        wupts = WuPointSet(points)
        i = 0
        for w in wupts:
            self.assertEqual(w.uuid(), uuids[i])
            self.assertEqual(wupts[uuids[i]].uuid(), uuids[i])
            self.assertAlmostEqual(w.utm.easting, eastings[i], places=3)
            self.assertAlmostEqual(w.utm.northing, northings[i], places=3)
            point_xy = w.toPointXY()
            self.assertAlmostEqual(point_xy.x, eastings[i], places = 3)
            self.assertAlmostEqual(point_xy.y, northings[i], places = 3)
            self.assertAlmostEqual(point_xy.z, 0.0, places = 3)
            i += 1
        self.assertEqual(i, 3)
        self.assertEqual(len(wupts), 3)

        bogus = '00000000-c433-5c42-be2e-fbd97ddff9ac'
        self.assertFalse(bogus in wupts)
        self.assertEqual(wupts.get(bogus), None)

        uu = uuids[1]
        self.assertTrue(uu in wupts)
        wpt = wupts[uu]
        self.assertEqual(wpt.uuid(), uu)
        self.assertNotEqual(wupts.get(uu), None)
        self.assertEqual(wupts.get(uu).uuid(), uu)

        # test index() function
        for i in xrange(len(uuids)):
            self.assertEqual(wupts.index(uuids[i]), i)
            self.assertEqual(wupts.points[i].id.uuid, uuids[i])

if __name__ == '__main__':
    import rosunit
    PKG='geodesy'
    rosunit.unitrun(PKG, 'test_xml_map_py', TestWuPoint) 
