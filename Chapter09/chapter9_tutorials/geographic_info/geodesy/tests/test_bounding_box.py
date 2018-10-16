#!/usr/bin/env python

import unittest

from geodesy.bounding_box import *     # module being tested

class TestPythonBoundingBox(unittest.TestCase):
    """Unit tests for Python bounding box functions. """

    def test_global_bounding_box(self):
        bb = makeGlobal()
        self.assertTrue(isGlobal(bb))

    def test_2d_bounding_box(self):
        min_lat = 30.3787400
        min_lon = -97.7344500
        max_lat = 30.3947700
        max_lon = -97.7230800
        bb = makeBounds2D(min_lat, min_lon, max_lat, max_lon)
        self.assertFalse(isGlobal(bb))
        self.assertTrue(is2D(bb))
        min_lat2, min_lon2, max_lat2, max_lon2 = getLatLong(bb)
        self.assertEqual(min_lat, min_lat2)
        self.assertEqual(min_lon, min_lon2)
        self.assertEqual(max_lat, max_lat2)
        self.assertEqual(max_lon, max_lon2)

    def test_3d_bounding_box(self):
        min_lat = 30.3787400
        min_lon = -97.7344500
        min_alt = 200.0
        max_lat = 30.3947700
        max_lon = -97.7230800
        max_alt = 400.0
        bb = makeBounds3D(min_lat, min_lon, min_alt,
                          max_lat, max_lon, max_alt)
        self.assertFalse(isGlobal(bb))
        self.assertFalse(is2D(bb))
        min_lat2, min_lon2, max_lat2, max_lon2 = getLatLong(bb)
        self.assertEqual(min_lat, min_lat2)
        self.assertEqual(min_lon, min_lon2)
        self.assertEqual(min_alt, bb.min_pt.altitude)
        self.assertEqual(max_lat, max_lat2)
        self.assertEqual(max_lon, max_lon2)
        self.assertEqual(max_alt, bb.max_pt.altitude)

if __name__ == '__main__':
    import rosunit
    PKG='geodesy'
    rosunit.unitrun(PKG, 'test_uuid_py', TestPythonBoundingBox) 
