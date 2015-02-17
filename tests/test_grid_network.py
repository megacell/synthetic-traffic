import unittest

import numpy as np

from networks.GridNetwork import GridNetwork
from sensors.SensorConfiguration import SensorConfiguration

__author__ = 'cathywu'

class TestGridNetwork(unittest.TestCase):

    def setUp(self):
        self.num_link = np.inf
        self.num_OD = np.inf
        self.num_cellpath_NB = 20
        self.num_cellpath_NL = 40
        self.num_cellpath_NS = 0
        self.num_linkpath = 100

        # GridNetwork doesn't support region cellpath sensors (for now)
        self.assertTrue(self.num_cellpath_NS==0)

    def test_concentrated(self):
        o_flow = 1
        concentration = 0.1

        TN = GridNetwork(o_flow=o_flow, concentration=concentration)

        S = SensorConfiguration(num_link=self.num_link, num_OD=self.num_OD,
                                num_cellpath_NB=self.num_cellpath_NB,
                                num_cellpath_NL=self.num_cellpath_NL,
                                num_cellpath_NS=self.num_cellpath_NS,
                                num_linkpath=self.num_linkpath)
        S.sample_sensors(TN)
        data = S.export_matrices(TN)
        self.assertTrue(True)

    def test_dispersed(self):
        o_flow = 1

        TN = GridNetwork(o_flow=o_flow)

        S = SensorConfiguration(num_link=self.num_link, num_OD=self.num_OD,
                                num_cellpath_NB=self.num_cellpath_NB,
                                num_cellpath_NL=self.num_cellpath_NL,
                                num_cellpath_NS=self.num_cellpath_NS,
                                num_linkpath=self.num_linkpath)
        S.sample_sensors(TN)
        data = S.export_matrices(TN)
        self.assertTrue(True)

if __name__ == '__main__':
    unittest.main()
