import ipdb
import random
import numpy as np

from synth_utils import to_sp

__author__ = 'cathywu'

class SensorConfiguration:

    def __init__(self, num_link=0, num_OD=0, num_cellpath_NB=0,
                 num_cellpath_NL=0, num_cellpath_NS=0, num_linkpath=0,
                 myseed=None, cp_thresh=5, cp_freq=10, bounding_box=None,
                 scale=None):
        self.num_link = num_link
        self.num_OD = num_OD
        self.num_linkpath = num_linkpath

        # Specific to cellpath sensors
        self.num_cellpath_NB = num_cellpath_NB
        self.num_cellpath_NL = num_cellpath_NL
        self.num_cellpath_NS = num_cellpath_NS # TODO add regions
        self.bounding_box = bounding_box

        # GridNetwork only
        self.cp_thresh = cp_thresh
        self.cp_freq = cp_freq
        # EquilibriumNetwork only
        self.scale = scale # EquilibriumNetwork only

        self.link_sensors, self.OD_sensors, self.cellpath_sensors, \
                self.linkpath_sensors = None, None, None, None

        # Save seed for reproducibility
        if myseed is None:
            myseed = random.randint(0,4294967295)
        np.random.seed(myseed)
        random.seed(myseed)
        self.myseed = myseed

    def sample_sensors(self, TN):
        if self.num_link > 0:
            self._sample_link_sensors(TN)
        if self.num_OD > 0:
            self._sample_OD_sensors(TN)
        if self.num_cellpath_NB+self.num_cellpath_NL+self.num_cellpath_NS > 0:
            self._sample_cellpath_sensors(TN)
        if self.num_linkpath > 0:
            self._sample_linkpath_sensors(TN)

    def _sample_link_sensors(self,TN):
        # TODO, NOOP, for now assumes all
        self.num_link = min(self.num_link,TN.num_links())

    def _sample_OD_sensors(self,TN):
        # TODO, NOOP, for now assumes all
        pass

    def _sample_cellpath_sensors(self,TN):
        from sensors.CellPath import CellPath
        self.num_cellpath_NL = min(self.num_cellpath_NL,TN.num_links())
        self.cp = CellPath(TN=TN,NB=self.num_cellpath_NB,
                            NL=self.num_cellpath_NL,NS=self.num_cellpath_NS,
                            freq=self.cp_freq,thresh=self.cp_thresh)
        self.cp.update_trajs(TN)

    def _sample_linkpath_sensors(self,TN):
        from sensors.LinkPath import LinkPath
        self.num_linkpath = min(self.num_linkpath,TN.num_links())
        self.lp = LinkPath(TN, N=self.num_linkpath)
        self.lp.update_trajs(TN)

    def export_matrices(self, TN):
        data = {}
        # Export A,b
        if self.num_link > 0:
            # FIXME common interface
            if TN.__class__.__name__ == 'EquilibriumNetwork':
                import networks.wardrop.path_solver as path_solver
                A_full = to_sp(path_solver.linkpath_incidence(TN.G))
                data['A'], data['b'] = A_full, A_full.dot(TN.p_flow)
            elif TN.__class__.__name__ == 'GridNetwork':
                from networks.grid_networks.static_matrix import generate_static_matrix_OD
                data['A'], data['b'] = generate_static_matrix_OD(TN, only_Ab=True)
            else:
                return NotImplemented
        # Export T,d
        if self.num_OD > 0:
            # FIXME common interface
            if TN.__class__.__name__ == 'EquilibriumNetwork':
                import networks.wardrop.path_solver as path_solver
                data['T'], data['d'] = path_solver.simplex(TN.G)
            else:
                data['T'], data['d'] = TN.simplex_od()
        # Export U,f
        if self.num_cellpath_NB+self.num_cellpath_NL+self.num_cellpath_NS > 0:
            data['U'], data['f'] = self.cp.simplex(TN)
        # Export V,g
        if self.num_linkpath > 0:
            data['V'], data['g'] = self.lp.simplex(TN)
        return data

    def simplify_matrices(self,data):
        pass

if __name__ == "__main__":
    import unittest
    from tests.test_sensor_configuration import TestSensorConfiguration
    unittest.main()
