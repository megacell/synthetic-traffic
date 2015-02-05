import unittest

__author__ = 'cathywu'

class TestEquilibriumNetwork(unittest.TestCase):
    def test_compile(self):
        import numpy as np
        from networks.EquilibriumNetwork import EquilibriumNetwork
        from sensors.SensorConfiguration import SensorConfiguration
        TN = EquilibriumNetwork()
        num_link = np.inf
        num_OD = np.inf
        num_cellpath_NB = 20
        num_cellpath_NL = 40
        num_cellpath_NS = 20
        num_linkpath = 100
        scale=0.2

        S = SensorConfiguration(num_link=num_link, num_OD=num_OD,
                                num_cellpath_NB=num_cellpath_NB,
                                num_cellpath_NL=num_cellpath_NL,
                                num_cellpath_NS=num_cellpath_NS,
                                num_linkpath=num_linkpath,
                                scale=scale)
        S.sample_sensors(TN)
        data = S.export_matrices(TN)
        self.assertTrue(True)

if __name__ == '__main__':
    unittest.main()
