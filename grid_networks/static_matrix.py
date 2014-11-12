import ipdb

import argparse
import collections
import random

import matplotlib
import networkx as nx
import numpy as np
import scipy.io

from GridNetwork import GridNetwork
from waypoints import Waypoints
import flows

def generate_static_matrix(grid, flow_from_each_node=1.0):
    # All route indices are with respect to _routes_.
    route_indices_by_origin = grid.get_route_indices_by_origin()

    b = np.array([grid.G[u][v]['flow'] for (u,v) in grid.sensors])

    xs = []
    ws = []
    As = []
    num_routes = []
    for node in grid.G.nodes():
        route_indices_from_node = route_indices_by_origin[node]
        edges_in_route = [collections.Counter(zip(grid.routes[i],
                                            grid.routes[i][1:])) for i in \
                          route_indices_from_node]

        x = np.zeros(shape=len(route_indices_from_node))
        w = np.zeros(shape=len(route_indices_from_node))
        A = np.zeros(shape=(len(grid.sensors), len(route_indices_from_node)))
        for j in xrange(len(route_indices_from_node)):
            x[j] = grid.flow_portions[route_indices_from_node[j]]
            route = grid.routes[route_indices_from_node[j]]
            w[j] = sum(1./grid.G[de[0]][de[1]]['weight'] for de in zip(route,
                                                                route[1:]))

            for i in xrange(len(grid.sensors)):
                if grid.sensors[i] in edges_in_route[j]:
                    A[i, j] = flow_from_each_node

        num_routes.append(len(route_indices_from_node))
        As.append(A)
        ws.append(w)
        xs.append(x)

    A,x,w = np.hstack(As), np.concatenate(xs), np.concatenate(ws)
    # FIXME need to regenerate b because some routes went over the same link
    # twice, which we aren't counting
    b = A.dot(x)

    return A, x, w, b, np.array(num_routes)

def generate_static_matrix_OD(grid):
    # All route indices are with respect to _routes_.
    route_indices_by_OD = grid.get_route_indices_by_OD()

    # link flow vector
    b = np.array([grid.G[u][v]['flow'] for (u,v) in grid.sensors])

    # initialize
    xs = []
    ws = []
    As = []
    num_routes = []

    # build x, w, A, num_routes (for L1 constraints)
    for origin in grid.G.nodes():
        for dest in grid.G.nodes():
            selected_route_indices_by_OD = route_indices_by_OD[origin][dest]
            edges_in_route = [collections.Counter(zip(grid.routes[i],
                                            grid.routes[i][1:])) for i in \
                              selected_route_indices_by_OD]
            # CAUTION: routes may double-count links for some reason

            # initialize
            x = np.zeros(shape=len(selected_route_indices_by_OD))
            w = np.zeros(shape=len(selected_route_indices_by_OD))
            A = np.zeros(shape=(len(grid.sensors),
                                len(selected_route_indices_by_OD)))

            # skip OD blocks that are all 0
            if grid.flow_OD[origin][dest] == 0:
                continue
            if grid.flow_OD[origin][dest] == []:
                continue

            # build A, x, w block by block (1 origin)
            for j in xrange(len(selected_route_indices_by_OD)):
                x[j] = grid.flow_portions[selected_route_indices_by_OD[j]]
                route = grid.routes[selected_route_indices_by_OD[j]]
                # TODO what is w?
                w[j] = sum(1./grid.G[u][v]['weight'] for (u,v) in zip(route,
                                                                    route[1:]))

                for i in xrange(len(grid.sensors)):
                    if grid.sensors[i] in edges_in_route[j]:
                        A[i, j] = flow_from_each_node[origin][dest]

            num_routes.append(len(selected_route_indices_by_OD))
            As.append(A)
            ws.append(w)
            xs.append(x)

    A,x,w = np.hstack(As), np.concatenate(xs), np.concatenate(ws)
    # FIXME need to regenerate b because some routes went over the same link
    # twice, which we aren't counting
    b = A.dot(x)

    return A, x, w, b, np.array(num_routes)

          
def generate_random_matrix(grid, flow_from_each_node=1.0):
    # All route indices are with respect to _routes_.
    route_indices_by_origin = grid.get_route_indices_by_origin()

    #  b = np.array([graph[u][v]['flow'] for (u,v) in sensors])

    xs = []
    ws = []
    num_routes = []
    for node in grid.G.nodes():
        route_indices_from_node = route_indices_by_origin[node]

        x = np.zeros(shape=len(route_indices_from_node))
        w = np.zeros(shape=len(route_indices_from_node))
        for j in xrange(len(route_indices_from_node)):
            x[j] = grid.flow_portions[route_indices_from_node[j]]
            route = grid.routes[route_indices_from_node[j]]
            w[j] = sum(1./grid.G[de[0]][de[1]]['weight'] for de in zip(route,
                                                                route[1:]))
        num_routes.append(len(route_indices_from_node))
        ws.append(w)
        xs.append(x)

    x, w = np.concatenate(xs), np.concatenate(ws)
    A = np.random.normal(size=(len(grid.sensors), len(grid.routes)))
    b = np.dot(A, x)

    return A, x, np.concatenate(ws), b, np.array(num_routes)

def assert_scaled_incidence(M):
    """
    Check that all column entries are either 0 or the same entry value
    :param M:
    :return:
    """
    m,n = M.shape
    col_sum = M.sum(axis=0)
    col_nz = (M > 0).sum(axis=0)
    entry_val = np.array([0 if M[:,i].nonzero()[0].size == 0 else \
                              M[M[:,i].nonzero()[0][0],i] for i in range(n)])
    assert (np.abs(col_sum - col_nz * entry_val) < 1e-10).all(), \
        'Not a proper scaled incidence matrix, check column entries'

def export_matrices(prefix, num_rows, num_cols, num_routes_per_od_pair,
                    num_nonzero_routes_per_o):

    # G = (V,E,w)
    grid = GridNetwork(num_cols=num_cols,num_rows=num_rows,
                       num_routes_per_od_pair=num_routes_per_od_pair)
    # (O,D),R,x
    grid.sample_OD_flow_sparse(1.0, num_nonzero_routes_per_o)
    grid.get_wp_trajs(10)
    U, r = Waypoints.simplex(grid, grid.wp_trajs)
    # TODO generate T,d
    # TODO generate x
    # TODO generate A,b
    # TODO generate V,g
    ipdb.set_trace()

    # static matrix considering origin flows
    A, x_true, w, b, num_routes = generate_static_matrix(grid)
    assert_scaled_incidence(A)
    scipy.io.savemat(prefix + 'small_graph.mat', {'A': A, 'x_true': x_true,
                'w': w, 'b': b, 'block_sizes': num_routes}, oned_as='column')

    # static matrix considering origin-destination flows
    A, x_true, w, b, num_routes = generate_static_matrix_OD(grid)
    assert_scaled_incidence(A)
    scipy.io.savemat(prefix + 'small_graph_OD.mat', {'A': A, 'x_true': x_true,
                'w': w, 'b': b, 'block_sizes': num_routes}, oned_as='column')

    # random matrix 'considering origin flows'
    A, x_true, w, b, num_routes = generate_random_matrix(grid)
    scipy.io.savemat(prefix + 'small_graph_random.mat', {'A': A,
                'x_true': x_true, 'w': w, 'b': b, 'block_sizes': num_routes},
                     oned_as='column')

    # same graph but dense OD blocks (CANNOT be used for comparison with above)
    grid.sample_OD_flow_dense(1.0, overall_sparsity=0.1)

    # static matrix considering origin-destination flows
    A, x_true, w, b, num_routes = generate_static_matrix_OD(grid)
    assert_scaled_incidence(A)
    scipy.io.savemat(prefix + 'small_graph_OD_dense.mat', {'A': A,
                'x_true': x_true, 'w': w, 'b': b, 'block_sizes': num_routes},
                     oned_as='column')

if __name__ == '__main__':

    import sys

    parser = argparse.ArgumentParser()
    parser.add_argument('--prefix', type=str,
                        help="Prefix where to export files to",
                        default='testtest_')
    parser.add_argument('--num_rows', type=int,
                        help='Number of rows in the road grid.', default=5)
    parser.add_argument('--num_cols', type=int,
                        help='Number of cols in the road grid.', default=5)
    parser.add_argument('--num_routes_per_od', type=int,
                        help='Number of routes per O-D pair.', default=2)
    parser.add_argument('--num_nonzero_routes_per_o', type=int,
                        help='Number of non-zero routes per origin.', default=2)
    args = parser.parse_args()
    #  num_rows = 5 if len(sys.argv) <= 1 else int(sys.argv[1])
    #  num_cols = 5 if len(sys.argv) <= 2 else int(sys.argv[2])
    #  num_routes_per_od_pair = 2 if len(sys.argv) <= 3 else int(sys.argv[3])
    #  num_nonzero_routes_per_origin

    export_matrices(args.prefix, args.num_rows, args.num_cols,
                    args.num_routes_per_od, args.num_nonzero_routes_per_o)
