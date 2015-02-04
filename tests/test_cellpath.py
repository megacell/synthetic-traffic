import unittest

__author__ = 'cathywu'

class TestCellPath(unittest.TestCase):
    def test_compile(self):
        from sensors.CellPath import CellPath
        from grid_networks.GridNetwork import GridNetwork
        cp = CellPath(NB=10,NL=100,NS=0)
        self.assertTrue(True)

if __name__ == '__main__':
    unittest.main()
