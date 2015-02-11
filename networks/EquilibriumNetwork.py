from __future__ import division, absolute_import

import ipdb
import scipy.io
import numpy.random as random
import numpy as np

import networks.wardrop.Graph as g
from TrafficNetwork import TrafficNetwork
from networks.wardrop.generate_paths import find_UESOpaths
import networks.wardrop.path_solver as path_solver

__author__ = 'jeromethai, cathywu'

class EquilibriumNetwork(TrafficNetwork):
    def __init__(self, type='LA-small', SO=False, demand=3,
                 delay_type='Polynomial', noise=0,
                 path='networks/los_angeles_data_2.mat'):
        TrafficNetwork.__init__(self)
        self.path = path
        self.noise = noise
        self.delay_type = delay_type
        if type == 'LA-small':
            from cvxopt import matrix as mat
            parameters = mat([0.0, 0.0, 0.0, 0.15])
            self.G = self.los_angeles(demand=demand,parameters=parameters)
            paths = find_UESOpaths(SO, path=path) # find the used paths in

            for p in paths:
                self.G.add_path_from_nodes(p)
            self.G.visualize(general=True)
            self.p_flow = path_solver.solver(self.G, update=True, SO=SO, random=random)
            # FIXME in this section, there is another dependence on los_angeles
            # in generate_graph, called through some sequence of functions
        elif type == 'LA-medium':
            # TODO (Jerome?)
            return NotImplemented
        else:
            return NotImplemented
        pass

    def num_links(self):
        return len(self.G.links)

    def los_angeles(self, demand=3, parameters=None):
        """Generate small map of L.A. with 122 links and 44 modes
        """

        if not self.path:
            self.path = 'networks/los_angeles_data_2.mat'
        data = scipy.io.loadmat(self.path)


        links = data['links']
        ODs1, ODs2, ODs3, ODs4 = data['ODs1'], data['ODs2'], data['ODs3'], data['ODs4']
        if self.noise>0.0:
            ODs1 = [(o, d, random.normal(f, self.noise*f)) for o,d,f in ODs1]
            ODs2 = [(o, d, random.normal(f, self.noise*f)) for o,d,f in ODs2]
            ODs3 = [(o, d, random.normal(f, self.noise*f)) for o,d,f in ODs3]
            ODs4 = [(o, d, random.normal(f, self.noise*f)) for o,d,f in ODs4]
            links = [(s, t, r, random.normal(d, self.noise*d), c) for s,t,r,d,c in links]

        nodes = data['nodes']
        tmp = links
        links = []

        if self.delay_type=='Polynomial':
            theta = parameters
            degree = len(theta)
            for startnode, endnode, route, ffdelay, slope in tmp:
                coef = [ffdelay*a*b for a,b in zip(theta, np.power(slope,
                                                        range(1,degree+1)))]
                links.append((startnode, endnode, route, ffdelay, (ffdelay,
                                                                slope, coef)))
        if self.delay_type=='Hyperbolic':
            a,b = parameters
            for startnode, endnode, route, ffdelay, slope in tmp:
                k1, k2 = a*ffdelay/slope, b/slope
                links.append((startnode, endnode, route, ffdelay, (ffdelay,
                                                            slope, k1, k2)))
        if self.delay_type=='None':
            for startnode, endnode, route, ffdelay, slope in tmp:
                links.append((startnode, endnode, route, ffdelay, None))

        g1 = g.create_graph_from_list(nodes, links, self.delay_type, ODs1, 'Map of L.A.')
        g2 = g.create_graph_from_list(nodes, links, self.delay_type, ODs2, 'Map of L.A.')
        g3 = g.create_graph_from_list(nodes, links, self.delay_type, ODs3, 'Map of L.A.')
        g4 = g.create_graph_from_list(nodes, links, self.delay_type, ODs4, 'Map of L.A.')
        return (g1, g2, g3, g4)[demand]

    def los_angeles_2(parameters=None, delaytype='None'):
        """Generate larger map of L.A. with 664 links and 194 nodes
        """
        # TODO (Jerome?)
        pass

    def get_region_weights(self):
        return [1],[(3.5, 0.5, 6.5, 3.0)]

if __name__ == "__main__":
    import unittest
    from tests.test_equilibrium_network import TestEquilibriumNetwork
    unittest.main()
