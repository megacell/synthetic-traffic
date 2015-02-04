import random
import numpy as np

class SensorConfiguration:

    def __init__(self, num_link=0, num_OD=0, num_cellpath_NB=0,
                 num_cellpath_NL=0, num_cellpath_NS=0, num_linkpath=0,
                 myseed=None, cp_thresh=5, cp_freq=10, bounding_box=None):
        self.num_link = num_link
        self.num_OD = num_OD
        self.num_linkpath = num_linkpath

        # Specific to cellpath sensors
        self.num_cellpath_NB = num_cellpath_NB
        self.num_cellpath_NL = num_cellpath_NL
        self.num_cellpath_NS = num_cellpath_NS # TODO add regions
        self.bounding_box = bounding_box
        self.cp_thresh = cp_thresh
        self.cp_freq = cp_freq

        self.link_sensors, self.OD_sensors, self.cellpath_sensors, \
                self.linkpath_sensors = None, None, None, None

        # Save seed for reproducibility
        if myseed is None:
            import sys
            myseed = random.randint(0,4294967295)
        np.random.seed(myseed)
        random.seed(myseed)
        self.myseed = myseed

    def sample_sensors(self, graph):
        if self.num_link >= 0:
            self._sample_link_sensors(graph)
        if self.num_OD >= 0:
            self._sample_OD_sensors(graph)
        if self.num_cellpath_NB+self.num_cellpath_NL+self.num_cellpath_NS >= 0:
            self._sample_cellpath_sensors(graph)
        if self.num_linkpath >= 0:
            self._sample_linkpath_sensors(graph)

    def _sample_link_sensors(self,graph):
        pass

    def _sample_OD_sensors(self,graph):
        pass

    def _sample_cellpath_sensors(self,graph):
        from waypoints.Waypoints import Waypoints # FIXME
        bbox = graph._get_bounding_box() # x1, y1, x2, y2
        cp = Waypoints(bbox=bbox)

        # uniformly sample points in bbox
        if self.num_cellpath_NB is not None:
            cp.uniform_random(n=self.num_cellpath_NB)

        # sample points along heavy edges (main roads)
        if self.num_cellpath_NL is not None:
            heavy_edges = graph._get_heavy_edges(thresh=self.cp_thresh)
            heavy_points =[(graph.G.node[e[0]]['pos'],graph.G.node[e[1]]['pos']) \
                           for e in heavy_edges]
            if len(heavy_points) > 1:
                cp.gaussian_polyline(heavy_points,n=self.num_cellpath_NL,tau=30)

        if cp.wp != {}:
            self.cp = cp
            self.path_cps, self.cp_trajs = cp._get_cp_trajs(graph,self.cp_freq)
            # TODO integrate path_cps, cp_trajs
        else:
            self.cp = None

    # def _sample_linkpath_sensors(self,graph):
    #     from sensors.LinkPath import LinkPath
    #     # TODO HERE IS WHERE I AM
    #     self.lp = LinkPath(graph, x_true, self.num_linkpath)

    def export_matrices(self, graph):
        data = None
        return data

if __name__ == "__main__":
    s = SensorConfiguration(num_link=5, num_OD=10, num_cellpath_NB=15, num_linkpath=20)
    from grid_networks.GridNetwork import GridNetwork
    G = GridNetwork()
    s.sample_sensors(G)
