import unittest

__author__ = 'cathywu'

class TestSensorConfiguration(unittest.TestCase):
    def test_grid(self):
        from sensors.SensorConfiguration import SensorConfiguration
        from grid_networks.GridNetwork import GridNetwork

        G = GridNetwork()
        G.sample_OD_flow()

        S = SensorConfiguration(num_link=5, num_OD=10, num_cellpath_NB=15, num_linkpath=20)
        S.sample_sensors(G)

        data = S.export_matrices(G)
        self.assertTrue(True)

    def test_eq(self):
        from sensors.SensorConfiguration import SensorConfiguration
        from EquilibriumNetwork import EquilibriumNetwork

        G = EquilibriumNetwork()

        S = SensorConfiguration(num_link=5, num_OD=10, num_cellpath_NB=15, num_linkpath=20)
        S.sample_sensors(G)

        data = S.export_matrices(G)
        self.assertTrue(True)

if __name__ == '__main__':
    unittest.main()
