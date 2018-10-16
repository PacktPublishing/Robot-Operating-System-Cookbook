#!/usr/bin/env python

import sys
import unittest

from geodesy.utm import *

## A sample python unit test
class TestUTMPoint(unittest.TestCase):

    def test_null_constructor(self):
        pt = UTMPoint()
        self.assertFalse(pt.valid(),
                         msg='uninitialized UTMPoint should be invalid: ' + str(pt))
        self.assertTrue(pt.is2D(),
                        msg='this UTMPoint should be 2D: ' + str(pt))
        self.assertRaises(ValueError, pt.toMsg)

    def test_wgs84_utm_conversion_3d(self):
        # UTM point in Pickle Research Campus, University of Texas, Austin
        ll = GeoPoint(latitude = 30.385315,
                      longitude = -97.728524,
                      altitude = 209.0)
        pt = fromMsg(ll)
        self.assertEqual(str(pt), 'UTM: [622159.338, 3362168.303, 209.000, 14R]',
                         msg='conversion failed: ' + str(pt))
        self.assertTrue(pt.valid(), msg='invalid UTMPoint: ' + str(pt))
        self.assertFalse(pt.is2D(), msg='this UTMPoint should be 3D: ' + str(pt))
        self.assertEqual(pt.gridZone(), (14, 'R'))
        self.assertEqual(str(pt.toMsg()), str(ll),
                         msg='GeoPoint conversion failed for: ' + str(pt))
        point_xyz = pt.toPoint()
        self.assertAlmostEqual(point_xyz.x, 622159.338, places = 3)
        self.assertAlmostEqual(point_xyz.y, 3362168.303, places = 3)
        self.assertAlmostEqual(point_xyz.z, 209.0, places = 3)

    def test_wgs84_utm_conversion_2d(self):
        # same point, but without altitude
        lat = 30.385315
        lon = -97.728524
        alt = float('nan')
        pt = fromLatLong(lat, lon)
        self.assertEqual(str(pt), 'UTM: [622159.338, 3362168.303, nan, 14R]',
                         msg='conversion failed: ' + str(pt))
        self.assertTrue(pt.valid(), msg='invalid UTMPoint: ' + str(pt))
        self.assertTrue(pt.is2D(), msg='this UTMPoint should be 2D: ' + str(pt))
        self.assertEqual(pt.gridZone(), (14, 'R'))
        self.assertEqual(str(pt.toMsg()), str(GeoPoint(lat, lon, alt)),
                         msg='GeoPoint conversion failed for: ' + str(pt))
        point_xy = pt.toPoint()
        self.assertAlmostEqual(point_xy.x, 622159.338, places = 3)
        self.assertAlmostEqual(point_xy.y, 3362168.303, places = 3)
        self.assertAlmostEqual(point_xy.z, 0.0, places = 3)
 
if __name__ == '__main__':
    import rosunit
    PKG='geodesy'
    rosunit.unitrun(PKG, 'test_utm_py', TestUTMPoint) 
