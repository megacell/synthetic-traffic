from __future__ import division

import ipdb

import networkx as nx
import itertools

import os, sys
lib_path = os.path.abspath('YenKSP')
sys.path.append(lib_path)
from YenKSP import graph
from YenKSP import algorithms
import collections

from waypoints.Waypoints import Waypoints
import flows

class GridNetwork:

    def __init__(self, num_cols=5, num_rows=5, num_routes_per_od_pair=2):
        # we have n*m nodes, (((of which a 5*5 grid is for Caltec and a 5*5 grid
        # is for the streets (for now) --> imagine it as a 5 rows, 10 columns
        # grid, indexed like a matrix)))

        # Generate directed road network
        self.n = num_cols
        self.m = num_rows
        self.r = num_routes_per_od_pair

        self.G, self.sensors = self.construct_grid()
        self.add_weights()
        self.add_inverse()

        H = self.invert_graph_weights()
        self.routes = self.pairwise_shortest_routes(H)

        self.wp = self.sample_waypoints()

        self.path_wps, self.wp_trajs = None, None
        self.nz_routes = None

    def construct_grid(self):
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

    def add_inverse(self):
        for (u, v, data) in self.G.edges(data=True):
            self.G.add_edge(v, u, weight = data['weight'])

        # Add opposite sensors
        sensors_opposite = [(v,u) for (u,v) in self.sensors]
        self.sensors.extend(sensors_opposite)

    def invert_graph_weights(self):
        # Invert graph weights
        H = graph.DiGraph()

        for (u, v) in self.G.edges():
            H.add_edge(u, v, cost = 1./self.G.edge[u][v]['weight'])
        return H

    def add_weights(self):
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

    def pairwise_shortest_routes(self, H):
        # Find k shortest routes between all 2 nodes
        k_shortests = []

        for pair in itertools.product(range(0, self.n*self.m), repeat=2):
            u,v = pair[0], pair[1]
            if u == v:
                continue
            # dijkstra would be routes.append(nx.shortest_path(G,source=v,target=w))
            k_shortest = algorithms.ksp_yen(H, u, v, max_k = self.r)
            for shortest in k_shortest:
                shortest['o'], shortest['d'], shortest['flow'] = u, v, 0
            k_shortests.extend(k_shortest)
        return k_shortests

    def get_bounding_box(self, margin=0.05):
        pos = nx.get_node_attributes(self.G,'pos')
        x, y = zip(*pos.values())
        x_min, x_max, y_min, y_max = min(x), max(x), min(y), max(y)
        w, h = x_max-x_min, y_max-y_min
        x1, x2, y1, y2 = x_min - w*margin, x_max + w*margin, y_min - h*margin,\
                         y_max + h*margin
        return x1, y1, x2, y2

    def get_route_indices_by_origin(self):
        route_indices_by_origin = collections.defaultdict(list)
        for i, r in enumerate(self.routes):
            route_indices_by_origin[r['o']].append(i)
        return route_indices_by_origin

    def new_dict_OD(self):
        dict_OD = collections.defaultdict(list)
        for i, r in enumerate(self.routes):
            dict_OD[r['o']] = collections.defaultdict(list)
        return dict_OD

    def get_route_indices_by_OD(self):
        route_indices_by_OD = self.new_dict_OD()
        for i, r in enumerate(self.routes):
            route_indices_by_OD[r['o']][r['d']].append(i)
        return route_indices_by_OD

    def get_OD_pairs(self):
        OD_pairs = collections.defaultdict(list)
        for i, r in enumerate(self.routes):
            OD_pairs[(r['path'][0],r['path'][-1])] = 1
        return OD_pairs.keys()

    def sample_waypoints(self):
        # TODO add arguments consistent with ISTTT for types of waypoints
        bbox = self.get_bounding_box()
        wp = Waypoints(bbox=bbox)
        wp.uniform_random(n=5)
        # print wp.closest_to_polyline([[0,0,2,2],[0,1,-1,2]],10)
        # print wp.closest_to_path(self.G, self.routes[10], 10)
        # ipdb.set_trace()

        # TODO: give routes of highways and stuff
        # wp.gaussian_polyline(n=5)
        # TODO: define regions?
        # wp.uniform_random_bbox()
        return wp

    def get_wp_trajs(self, n, fast=False, tol=1e-3):
        self.path_wps, self.wp_trajs = self.wp.get_wp_trajs(self.G,self.routes,
                                self.nz_routes, n, fast=fast, tol=tol)

    def sample_OD_flow_sparse(self, flow_from_each_node=1.0,
                            num_nonzero_routes=2):
        self.G, self.routes = flows.annotate_with_flows(self,
                            flow_from_each_node=flow_from_each_node,
                            num_nonzero_routes=num_nonzero_routes)
        self.update_nz_routes()

    def sample_OD_flow_dense(self, flow_from_each_node=1.0,
                                         overall_sparsity=0.1):
        self.G, self.routes = flows.annotate_with_flows_dense_blocks(self,
                            flow_from_each_node=1.0, overall_sparsity=0.1)
        self.update_nz_routes()

    def update_flows(self, flow_portions, flow_from_each_node, r_ind, r_weights):
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
                node['2nd_flow'][(p, s)] = (current_flow + flow_from_each_node * w,
                                            current_routes | set([i]))

    def update_nz_routes(self, tol=1e-3):
        self.nz_routes = [i for (i,r) in enumerate(self.routes) if r['flow'] > tol]

    def draw_graph(self):
        pos = nx.get_node_attributes(self.G,'pos')

        nx.draw(self.G,pos)

        edge_labels=dict([((u,v,),d['weight'])
                          for u,v,d in self.G.edges(data=True)])

        nx.draw_networkx_edge_labels(self.G, pos, edge_labels=edge_labels)

        nx.draw_networkx_edges(self.G, pos, edgelist=self.sensors, width=3,
                               alpha=0.5, edge_color='b')

        import matplotlib.pyplot
        matplotlib.pyplot.show()

if __name__ == '__main__':

    G = GridNetwork()

    # print G.sensors
    # print len(G.sensors)
    # print G.routes

    G.draw_graph()

