import unittest

__author__ = 'cathywu'

class TestSensorConfiguration(unittest.TestCase):
    def test_compile(self):
        from sensors.SensorConfiguration import SensorConfiguration
        from grid_networks.GridNetwork import GridNetwork
        s = SensorConfiguration(num_link=5, num_OD=10, num_cellpath_NB=15, num_linkpath=20)
        G = GridNetwork()
        s.sample_sensors(G)
        data = s.export_matrices(G)
        self.assertTrue(True)

if __name__ == '__main__':
    unittest.main()
