import unittest

from sensors.SensorConfiguration import SensorConfiguration

__author__ = 'cathywu'

class TestSensorConfiguration(unittest.TestCase):
    # TODO WARNING does not test cellpath NB sensors

    def setUp(self):
        from GridNetwork import GridNetwork
        self.TN1 = GridNetwork()
        self.TN1.sample_OD_flow()

        from EquilibriumNetwork import EquilibriumNetwork
        self.TN2 = EquilibriumNetwork()

    def test_inf(self):
        import numpy as np
        S = SensorConfiguration(num_link=np.inf, num_OD=np.inf,
                                num_cellpath_NB=100, num_cellpath_NL=np.inf,
                                num_linkpath=np.inf)

        S.sample_sensors(self.TN1)
        data = S.export_matrices(self.TN1)
        S.sample_sensors(self.TN2)
        data = S.export_matrices(self.TN2)
        self.assertTrue(True)

    def test_basic(self):
        S = SensorConfiguration(num_link=5, num_OD=10, num_cellpath_NB=4,
                                num_cellpath_NL=5, num_linkpath=5)

        S.sample_sensors(self.TN1)
        data = S.export_matrices(self.TN1)
        S.sample_sensors(self.TN2)
        data = S.export_matrices(self.TN2)
        self.assertTrue(True)

    def test_zero(self):
        S = SensorConfiguration(num_link=0, num_OD=0, num_cellpath_NB=0,
                                num_cellpath_NL=0, num_linkpath=0)

        S.sample_sensors(self.TN1)
        data = S.export_matrices(self.TN1)
        S.sample_sensors(self.TN2)
        data = S.export_matrices(self.TN2)
        self.assertTrue(True)

if __name__ == '__main__':
    unittest.main()
