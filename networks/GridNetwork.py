from __future__ import division

import ipdb
import os, sys
import itertools
import logging
import random
import collections

import networkx as nx
import numpy as np
from scipy.sparse import csr_matrix, coo_matrix
import matplotlib.pyplot as plt

lib_path = os.path.abspath('YenKSP')
sys.path.append(lib_path)
from YenKSP import graph
from YenKSP import algorithms

from networks.TrafficNetwork import TrafficNetwork
from synth_utils import to_np

__author__ = 'cathywu'

class GridNetwork(TrafficNetwork):

    def __init__(self, ncol=5, nrow=5, nodroutes=2, myseed=None, o_flow=1.0,
                 nnz_oroutes=2, concentration=None):
        TrafficNetwork.__init__(self)
        # we have n*m nodes, (((of which a 5*5 grid is for Caltec and a 5*5 grid
        # is for the streets (for now) --> imagine it as a 5 rows, 10 columns
        # grid, indexed like a matrix)))

        # Generate directed road network
        self.n, self.m, self.r = ncol, nrow, nodroutes
        self.G, self.sensors = self._construct_grid()
        self._add_weights()
        self._add_reverse()
        logging.debug('Graph generated')

        # Generate routes
        H = self._invert_graph_weights()
        self.routes = self._pairwise_shortest_routes(H)
        logging.debug('Routes generated')

        # Routes with non-zero flow
        self.nz_routes = None

        self.bbox = self.get_bounding_box()

        # Save seed for reproducibility
        if myseed is None:
            myseed = random.randint(0,4294967295)
        np.random.seed(myseed)
        random.seed(myseed)
        self.myseed = myseed

        self.o_flow = o_flow
        self.concentration = concentration
        self.nnz_oroutes = nnz_oroutes
        self.sample_OD_flow()


    def num_links(self):
        return len(self.G.edges())

    def get_route_flow(self,i):
        return self.routes[i]['flow']

    def _construct_grid(self):
        sensors = []
        G = nx.DiGraph()

        for j in range(0, self.n):
            for k in range(0, self.m):
                G.add_node(k * self.n + j, pos = (j, -k))

        for j in range(0, self.n-1):
            for k in range(0, self.m-1):
                G.add_edge(k * self.n + j, k * self.n + j + 1, weight=1)
                sensors.append((k * self.n + j, k * self.n + j + 1))
                G.add_edge(k * self.n + j, (k+1) * self.n + j, weight=1)
                sensors.append((k * self.n + j, (k+1) * self.n + j))

        # Add edges to the last row
        for j in range(0, self.n-1):
            G.add_edge((self.m-1)*self.n+j,(self.m-1)*self.n+j+1, weight=1)
            sensors.append(((self.m-1)*self.n + j, (self.m-1)*self.n + j + 1))
            # had to do this for this small node case to get at least 2 highways...

        # Add edges to the last column
        for k in range(0, self.m-1):
            G.add_edge(k*self.n + self.n-1, (k+1)*self.n + self.n-1, weight=1)
            sensors.append((k * self.n + self.n-1, (k+1) * self.n + self.n-1))

        return G, sensors

    def _add_reverse(self):
        """
        Add reverse edges of sensors to graph
        :return:
        """
        for (u, v, data) in self.G.edges(data=True):
            self.G.add_edge(v, u, weight = data['weight'])

        # Add opposite sensors
        sensors_opposite = [(v,u) for (u,v) in self.sensors]
        self.sensors.extend(sensors_opposite)

    def _invert_graph_weights(self):
        # Invert graph weights
        H = graph.DiGraph()
        H._data = {} # FIXME reset data, not entirely sure why this is necessary

        for (u, v) in self.G.edges():
            H.add_edge(u, v, cost = 1./self.G.edge[u][v]['weight'])
        return H

    def _add_weights(self):
        # Set bigger road weights
        for k in range(0, self.m-1):
            for j in range(0, self.n-1):
                # Highways, every 4 rows
                if k % 4 == 0:
                    self.G.edge[k*self.n + j][k*self.n + j+1]['weight'] = 6
                    # sensors.append((k*n + j,k*n + j+1))

                # Big streets, every other 4 rows
                if k % 4 == 2:
                    self.G.edge[k*self.n + j][k*self.n + j+1]['weight'] = 3
                    # Philipp: Hungry for sensors
                    # if j % 3 == 0:
                    # sensors.append((k*n + j,k*n + j+1))

                # Half big streets, every 2 columns
                if j % 2 == 0:
                    self.G.edge[k*self.n + j][(k+1)*self.n + j]['weight'] = 2
                    # oh my gosh, I want more sensors
                    # sensors.append((k*n + j,(k+1)*n + j))

    @staticmethod
    def has_duplicate(path):
        return len(set(path)) == len(path)

    def _pairwise_shortest_routes(self, H):
        # Find k shortest routes between all 2 nodes
        k_shortests = []

        for pair in itertools.product(range(0, self.n*self.m), repeat=2):
            u,v = pair[0], pair[1]
            if u == v:
                continue
            # dijkstra would be routes.append(nx.shortest_path(G,source=v,target=w))
            k_shortest = algorithms.ksp_yen(H, u, v, max_k = self.r)
            saved_routes = []
            for shortest in k_shortest:
                if GridNetwork.has_duplicate(shortest['path']):
                    k_shortest.remove(shortest)
                shortest['o'], shortest['d'], shortest['flow'] = u, v, 0
                saved_routes.append(shortest)
            k_shortests.extend(saved_routes)
        return k_shortests

    def get_route_indices_by_origin(self):
        route_indices_by_origin = collections.defaultdict(list)
        for i, r in enumerate(self.routes):
            route_indices_by_origin[r['o']].append(i)
        return route_indices_by_origin

    def _new_dict_OD(self):
        dict_OD = collections.defaultdict(list)
        for i, r in enumerate(self.routes):
            dict_OD[r['o']] = collections.defaultdict(list)
        return dict_OD

    def get_route_indices_by_OD(self):
        route_indices_by_OD = self._new_dict_OD()
        for i, r in enumerate(self.routes):
            route_indices_by_OD[r['o']][r['d']].append(i)
        return route_indices_by_OD

    def _get_OD_pairs(self):
        OD_pairs = collections.defaultdict(list)
        for i, r in enumerate(self.routes):
            OD_pairs[(r['path'][0],r['path'][-1])] = 1
        return OD_pairs.keys()

    # SAMPLE VARIOUS FLOWS (HELPER)
    # --------------------------------------------------------------------------
    def _get_heavy_edges(self, thresh=5):
        heavy = []
        for o,do in self.G.edge.items():
            for d,v in do.items():
                if v['weight'] >= thresh:
                    heavy.append((o,d))
        return heavy

    def get_heavy_points(self, thresh=5):
        heavy_edges = self._get_heavy_edges(thresh=thresh)
        heavy_points =[(self.G.node[e[0]]['pos'],self.G.node[e[1]]['pos']) \
                       for e in heavy_edges]
        return heavy_points

    def get_bounding_box(self, margin=0.05):
        pos = nx.get_node_attributes(self.G,'pos')
        x, y = zip(*pos.values())
        x_min, x_max, y_min, y_max = min(x), max(x), min(y), max(y)
        w, h = x_max-x_min, y_max-y_min
        x1, x2, y1, y2 = x_min - w*margin, x_max + w*margin, y_min - h*margin, \
                         y_max + h*margin
        return x1, y1, x2, y2

    # SAMPLE VARIOUS FLOWS
    # --------------------------------------------------------------------------
    def sample_OD_flow(self):
        if self.concentration is None:
            self._sample_flows(nnz_oroutes=self.nnz_oroutes)
        else:
            self._sample_flows_dense(concentration=self.concentration)
        self._update_nz_routes()

    def _sample_flows(self, nnz_oroutes=2):
        '''Generate traffic from each origin onto some small fraction of its routes, \
                and compute the amount of flow at each edge.'''

        # collect routes by origin or by OD pair
        # Note: All route indices are with respect to _routes_.
        route_indices_by_origin = self.get_route_indices_by_origin()
        route_indices_by_OD = self.get_route_indices_by_OD()

        flow_portions = [0] * len(self.routes) # from origin
        flow_portions_OD = [0] * len(self.routes) # from origin to destination
        self.od_flows = self._new_dict_OD()

        # initialize the flows, in case a node is not in the interior of any route
        for n in self.G.nodes():
            self.G.node[n]['2nd_flow'] = {}

        # initialize all flows on edges to 0
        for (u,v) in self.G.edges():
            self.G.edge[u][v]['flow'] = 0

        # sample routes and compute aggregate first-order, second-order info on
        # origins and origin-destination pairs

        for origin in self.G.nodes():
            # consider all routes out of origin
            route_indices_from_origin = route_indices_by_origin[origin]
            #    num_nonzero_routes = max(1, int(sparsity * len(route_indices_from_origin)))
            # select routes with non-zero flow
            selected_route_indices = sorted(random.sample(route_indices_from_origin,
                            min(nnz_oroutes,len(route_indices_from_origin))))
            # probability prior (uniform dirichlet) on non-zero flow routes
            selected_route_weights = np.random.dirichlet([1] * nnz_oroutes,
                                                            1)[0]

            self._update_flows(flow_portions, self.o_flow,
                              selected_route_indices,selected_route_weights)

            # normalize flows for each OD pair
            for dest in self.G.nodes():
                # normalize weights per OD pair
                selected_route_indices_OD = route_indices_by_OD[origin][dest]
                total = sum(flow_portions[i] for i in selected_route_indices_OD)
                self.od_flows[origin][dest] = total
                if total == 0:
                    continue
                for i in selected_route_indices_OD:
                    flow_portions_OD[i] = flow_portions[i]/total

    def _sample_flows_dense(self, concentration=0.1):
        '''Generate traffic from each origin onto some small fraction of its routes, \
                and compute the amount of flow at each edge.'''

        # collect routes by origin or by OD pair
        # Note: All route indices are with respect to _routes_.
        route_indices_by_OD = self.get_route_indices_by_OD()

        flow_portions_OD = [0] * len(self.routes) # from origin to destination
        self.od_flows = self._new_dict_OD()
        OD_pairs = self._get_OD_pairs()

        # initialize the flows, in case a node is not in the interior of any route
        for n in self.G.nodes():
            self.G.node[n]['2nd_flow'] = {}

        # initialize all flows on edges to 0
        for (u,v) in self.G.edges():
            self.G.edge[u][v]['flow'] = 0

        # sample OD pairs
        selected_OD_pairs = random.sample(OD_pairs,int(len(OD_pairs) * concentration))
        for origin,dest in selected_OD_pairs:
            self.od_flows[origin][dest] = self.o_flow
            selected_route_indices = route_indices_by_OD[origin][dest]
            num_routes = len(selected_route_indices)
            selected_route_weights = np.random.dirichlet([1] * num_routes, 1)[0]

            self._update_flows(flow_portions_OD, self.o_flow,
                              selected_route_indices,selected_route_weights)

    # FLOW UPDATE
    # --------------------------------------------------------------------------
    def _update_od_flows(self, od_flows):
        self.od_flows = od_flows

    def _update_flows(self, flow_portions, flow_from_each_node, r_ind, r_weights):
        """
        Update route flows, compute link flows and split ratios
        :param flow_portions:
        :param flow_from_each_node:
        :param r_ind:
        :param r_weights:
        :return:
        """
        for i, w in zip(r_ind, r_weights):
            flow_portions[i] = w
            self.routes[i]['flow'] = flow_from_each_node * w

            # add up flows on each link
            for u, v in zip(self.routes[i]['path'], self.routes[i]['path'][1:]):
                self.G.edge[u][v]['flow'] += flow_from_each_node * w

            # add up "turn" information on each transition
            # p = predecessor, n = node, s = successor
            for p, n, s in zip(self.routes[i]['path'], self.routes[i]['path'][1:],
                               self.routes[i]['path'][2:]):
                # add "second order flow"
                node = self.G.node[n]
                current_flow = node['2nd_flow'][(p, s)][0] if (p, s) in \
                                                    node['2nd_flow'] else 0
                current_routes = node['2nd_flow'][(p, s)][1] if (p, s) in \
                                                    node['2nd_flow'] else set()
                node['2nd_flow'][(p, s)] = (current_flow+flow_from_each_node*w,
                                            current_routes | set([i]))

    def _update_nz_routes(self, tol=1e-3):
        self.nz_routes = [i for (i,r) in enumerate(self.routes) if r['flow']>tol]

    # SIMPLEX
    # --------------------------------------------------------------------------
    def simplex_od(self):
        """Build simplex constraints from od flows
        """
        from cvxopt import matrix, spmatrix
        rids = self.get_route_indices_by_OD()
        n = sum([len(v) for v in self.od_flows.values()])
        m = len(self.routes)
        I, J, d, k = [], [], matrix(0.0, (n,1)), 0
        for i in self.od_flows.keys():
            for j, od_flow in self.od_flows[i].iteritems():
                d[k] = od_flow
                for rid in rids[i][j]:
                    I.append(k)
                    J.append(rid)
                k += 1
        T = coo_matrix(([1.0] * len(I),(I,J)), shape=(n,m)).tocsr()
        d = to_np(d)
        return T, d

    # ART
    # --------------------------------------------------------------------------
    def draw_graph(self):
        # draw nodes and edges
        pos = nx.get_node_attributes(self.G,'pos')
        nx.draw(self.G,pos)
        plt.hold()

        # draw edge weights
        edge_to_weight=dict([((u,v,),d['weight']) for u,v,d in \
                             self.G.edges(data=True)])
        from collections import defaultdict
        dd = defaultdict(list)
        for k, v in edge_to_weight.iteritems():
            dd[v].append(k)
        weight_to_edge = dict(dd)
        for k,v in weight_to_edge.iteritems():
            nx.draw_networkx_edges(self.G, pos, edgelist=v, width=k,
                                   alpha=0.5, edge_color='k')
        # draw sensors
        nx.draw_networkx_edges(self.G, pos, edgelist=self.sensors, width=1,
                               alpha=0.5, edge_color='b')

if __name__ == '__main__':
    import unittest
    from tests.test_grid_network import TestGridNetwork
    unittest.main()

    # G = GridNetwork()

    # G.draw_graph()

