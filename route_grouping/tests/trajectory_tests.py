import unittest

from django.contrib.gis.geos import LineString

from route_grouping.trajectory import Trajectory


class TestTrajectory(unittest.TestCase):
    def setUp(self):
        # create some lines
        id_to_geometry = dict()
        id_to_geometry[0] = LineString([(0,0),(0,1)],srid=1)
        id_to_geometry[1] = LineString([(0,1),(0,2)],srid=1)
        id_to_geometry[2] = LineString([(0,2),(0,3)],srid=1)
        id_to_geometry[3] = LineString([(0,3),(0,4),(4,7)],srid=1)

        # get their lengths
        length_cache = {k:id_to_geometry[k].length for k in id_to_geometry.keys()}

        self.id_to_geometry = id_to_geometry
        self.length_cache = length_cache

    def make_trajectory(self, seq):
        return Trajectory(seq, None, self.id_to_geometry, self.length_cache)

    def test_length(self):
        t = self.make_trajectory([1,2])
        self.assertAlmostEqual(t.length(), 2, delta=.0001)

    def test_length_complex(self):
        t = self.make_trajectory([3])
        self.assertAlmostEqual(t.length(), 6, delta=.0001)

    def test_match_percent(self):
        t1 = self.make_trajectory([0,1])
        t2 = self.make_trajectory([0])
        self.assertAlmostEqual(t1.match_percent(t2), 0.5, delta=.0001)

    def test_match_percent_no_overlapping_segments(self):
        t1 = self.make_trajectory([0])
        t2 = self.make_trajectory([1])
        self.assertAlmostEqual(t1.match_percent(t2), 0, delta=.0001)

    def test_match_percent_all_overlapping_segments(self):
        t1 = self.make_trajectory([0])
        t2 = self.make_trajectory([0])
        self.assertAlmostEqual(t1.match_percent(t2), 1, delta=.0001)

if __name__ == '__main__':
    unittest.main()