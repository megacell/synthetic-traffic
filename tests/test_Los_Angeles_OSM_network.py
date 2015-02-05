import unittest

__author__ = 'cathywu'

class TestLosAngelesOSMNetwork(unittest.TestCase):
    def test_compile(self):
        from networks.LosAngelesOSMNetwork import LosAngelesOSMNetwork
        from sensors.CellPath import CellPath
        rs = [0.25, 0.5]

        for r in rs:
            LA = LosAngelesOSMNetwork()
            cp = CellPath(NB=150*r,NL=50*r,NS=800*r,TN=LA)
        self.assertTrue(True)

if __name__ == '__main__':
    unittest.main()
