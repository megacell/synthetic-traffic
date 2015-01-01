import ipdb

import argparse
import collections
import random

import matplotlib
import networkx as nx
import numpy as np
import scipy.io
import scipy.sparse as sps
import logging

from GridNetwork import GridNetwork
from waypoints import Waypoints

# Clean array wrapper
def array(x):
    return np.atleast_1d(np.squeeze(np.array(x)))

# Clean sparse matrix wrapper
def sparse(A):
    if type(A) == np.ndarray:
        return sps.csr_matrix(A)
    return A.tocsr()

def sps2array(x):
    return array(x.todense())

def assert_simplex_incidence(M,n):
    """
    1. Check that the width of the matrix is correct.
    2. Check that each column sums to 1
    3. Check that there are exactly n nonzero values
    :param M:
    :param n:
    :return:
    """
    assert M.shape[1] == n, 'Incidence matrix: wrong size'
    assert (M.sum(axis=0)-1).nonzero()[0].size == 0, \
        'Incidence matrix: columns should sum to 1'
    assert M.nnz == n, 'Incidence matrix: should be n nonzero values'

def assert_scaled_incidence(M):
    """
    Check that all column entries are either 0 or the same entry value

    :param M:
    :return:
    """
    if len(M.shape) == 1:
        M = M.reshape((M.size,1))
    n = M.shape[1]
    col_sum = M.sum(axis=0)
    col_nz = (M > 0).sum(axis=0)
    entry_val = np.array([0 if M[:,i].nonzero()[0].size == 0 else \
                              M[M[:,i].nonzero()[0][0],i] for i in range(n)])
    assert (np.abs(array(col_sum) - array(col_nz) * entry_val) < 1e-10).all(), \
        'Not a proper scaled incidence matrix, check column entries'

def validate_data(A,b,x_true,U=None,f=None,T=None,d=None,V=None,g=None,n=0):
    assert_scaled_incidence(A)
    assert np.linalg.norm(A.dot(x_true) - b) == 0, 'Ax != b'
    if U is not None and f is not None:
        assert_simplex_incidence(U, n)
        assert np.linalg.norm(U.dot(x_true) - f) == 0, 'Ux != r'
    if T is not None and d is not None:
        assert_simplex_incidence(T, n)
        assert np.linalg.norm(T.dot(x_true) - d) == 0, 'Tx != d'
    if V is not None and g is not None:
        assert np.linalg.norm(V.dot(x_true) - g) == 0, 'Vx != g'

def generate_static_matrix(grid, flow_from_each_node=1.0):
    # All route indices are with respect to _routes_.
    route_indices_by_origin = grid.get_route_indices_by_origin()

    # link flow vector
    # b = np.array([grid.G[u][v]['flow'] for (u,v) in grid.sensors])

    # initialize
    xs, ws, As, num_routes = [], [], [], []
    for node in grid.G.nodes():
        route_indices_from_node = route_indices_by_origin[node]
        edges_in_route = [collections.Counter(zip(grid.routes[i]['path'],
                                    grid.routes[i]['path'][1:])) for i in \
                          route_indices_from_node]

        x = np.zeros(shape=len(route_indices_from_node))
        w = np.zeros(shape=len(route_indices_from_node))
        A = np.zeros(shape=(len(grid.sensors), len(route_indices_from_node)))
        for j in xrange(len(route_indices_from_node)):
            x[j] = grid.routes[route_indices_from_node[j]]['flow']
            route = grid.routes[route_indices_from_node[j]]['path']
            w[j] = sum(1./grid.G[de[0]][de[1]]['weight'] for de in \
                       zip(route, route[1:]))

            for i in xrange(len(grid.sensors)):
                if grid.sensors[i] in edges_in_route[j]:
                    A[i, j] = 1

        num_routes.append(len(route_indices_from_node))
        As.append(A)
        ws.append(w)
        xs.append(x)
    A,x,w = np.hstack(As), np.concatenate(xs), np.concatenate(ws)
    # FIXME need to regenerate b because some routes went over the same link
    # twice, which we aren't counting
    b = A.dot(x)

    T, d = grid.simplex_od()
    U, f = grid.simplex_cp()
    V, g = grid.simplex_lp() if grid.lp is not None else (None, None)

    return A, x, w, b, T, d, U, f, V, g, np.array(num_routes)

def generate_static_matrix_OD(grid):
    # All route indices are with respect to _routes_.
    route_indices_by_OD = grid.get_route_indices_by_OD()

    # link flow vector
    # b = np.array([grid.G[u][v]['flow'] for (u,v) in grid.sensors])

    # initialize
    xs, ws, As, num_routes = [], [], [], []

    # build x, w, A, num_routes (for L1 constraints)
    for origin in grid.G.nodes():
        for dest in grid.G.nodes():
            if origin not in grid.od_flows or dest not in grid.od_flows[origin]:
                pass

            selected_route_indices_by_OD = route_indices_by_OD[origin][dest]
            edges_in_route = [collections.Counter(zip(grid.routes[i]['path'],
                                        grid.routes[i]['path'][1:])) for i in \
                              selected_route_indices_by_OD]
            # CAUTION: routes may double-count links for some reason

            # initialize
            x = np.zeros(shape=len(selected_route_indices_by_OD))
            w = np.zeros(shape=len(selected_route_indices_by_OD))
            A = np.zeros(shape=(len(grid.sensors),
                                len(selected_route_indices_by_OD)))

            # skip OD blocks that are all 0
            # if grid.od_flows[origin][dest] == 0:
            #     continue
            # if grid.od_flows[origin][dest] == []:
            #     continue

            # build A, x, w block by block (1 origin)
            for j in xrange(len(selected_route_indices_by_OD)):
                x[j] = grid.routes[selected_route_indices_by_OD[j]]['flow']
                route = grid.routes[selected_route_indices_by_OD[j]]['path']
                # TODO what is w?
                w[j] = sum(1./grid.G[u][v]['weight'] for (u,v) in zip(route,
                                                                    route[1:]))

                for i in xrange(len(grid.sensors)):
                    if grid.sensors[i] in edges_in_route[j]:
                        A[i, j] = 1

            num_routes.append(len(selected_route_indices_by_OD))
            As.append(A)
            ws.append(w)
            xs.append(x)

    A,x,w = np.hstack(As), np.concatenate(xs), np.concatenate(ws)
    # FIXME need to regenerate b because some routes went over the same link
    # twice, which we aren't counting
    b = A.dot(x)

    T, d = grid.simplex_od()
    U, f = grid.simplex_cp()
    V, g = grid.simplex_lp() if grid.lp is not None else (None, None)

    return A, x, w, b, T, d, U, f, V, g, np.array(num_routes)

          
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
            x[j] = grid.routes[route_indices_from_node[j]]['flow']
            route = grid.routes[route_indices_from_node[j]]['path']
            w[j] = sum(1./grid.G[de[0]][de[1]]['weight'] for de in zip(route,
                                                                route[1:]))
        num_routes.append(len(route_indices_from_node))
        ws.append(w)
        xs.append(x)

    x, w = np.concatenate(xs), np.concatenate(ws)
    A = np.random.normal(size=(len(grid.sensors), len(grid.routes)))
    b = np.dot(A, x)

    return A, x, np.concatenate(ws), b, np.array(num_routes)

def export_matrices(prefix, nrow, ncol, nodroutes=5, nnz_oroutes=2, NB=60,
                 NS=20, NL=15, NLP=20, export=True, type='all'):

    # G = (V,E,w)
    grid = GridNetwork(ncol=ncol, nrow=nrow, nodroutes=nodroutes, NB=NB, NS=NS,
                       NL=NL, NLP=NLP)
    # (O,D),R,x
    if type == 'all' or type == 'small_graph.mat' or \
                    type == 'small_graph_OD.mat' or \
                    type == 'small_graph_random.mat':
        grid.sample_OD_flow(o_flow=1.0, nnz_oroutes=nnz_oroutes)
    # TODO generate T,d
    # TODO generate V,g

    if type == 'all' or type == 'small_graph.mat':
        # static matrix considering origin flows
        A, x_true, w, b, T, d, U, f, V, g, num_routes = generate_static_matrix(grid)
        validate_data(A,b,x_true,U=U,f=f,T=T,d=d,V=V,g=g,n=len(grid.routes))
        data = { 'A': A, 'x_true': x_true, 'w': w, 'b': b, 'T':T, 'd':d, 'U':U,
                 'f':f }
        if V is not None and g is not None:
            data['V'], data['g'] = V, g
        if export:
            scipy.io.savemat(prefix + 'small_graph.mat', data, oned_as='column')

    if type == 'all' or type == 'small_graph_OD.mat':
        # static matrix considering origin-destination flows
        A,x_true,w,b,T,d,U,f,V,g,num_routes = generate_static_matrix_OD(grid)
        validate_data(A,b,x_true,U=U,f=f,T=T,d=d,V=V,g=g,n=len(grid.routes))
        data = { 'A': A, 'x_true': x_true, 'w': w, 'b': b, 'T':T, 'd':d, 'U':U,
                 'f':f }
        if V is not None and g is not None:
            data['V'], data['g'] = V, g
        if export:
            scipy.io.savemat(prefix + 'small_graph_OD.mat', data, oned_as='column')

    if type == 'all' or type == 'small_graph_random.mat':
        # random matrix 'considering origin flows'
        A,x_true,w,b,num_routes = generate_random_matrix(grid)
        data = { 'A': A,'x_true': x_true, 'w': w, 'b': b, 'block_sizes': num_routes,
                    'U': U, 'f': f }
        if export:
            scipy.io.savemat(prefix + 'small_graph_random.mat', data, oned_as='column')

    if type == 'all' or type == 'small_graph_OD_dense.mat':
        # same graph but dense OD blocks (CANNOT be used for comparison with above)
        grid.sample_OD_flow(o_flow=1.0, sparsity=0.1)

        # static matrix considering origin-destination flows
        A,x_true,w,b,T,d,U,f,V,g,num_routes = generate_static_matrix_OD(grid)
        nz = array(T.sum(axis=0).nonzero()[1])
        z = array((T.sum(axis=0)-1).nonzero()[1])
        if T.size == 0:
            logging.error('T is empty, quitting')
            return {'error' : 'T is empty'}
        T, A, U, x_true = T[:,nz], A[:,nz], U[:,nz], x_true[nz]
        b, d = A.dot(x_true), T.dot(x_true)
        if V is not None and g is not None:
            V = V[:,nz]
            g = V.dot(x_true)
        validate_data(A,b,x_true,U=U,f=f,V=V,g=g,n=nz.size)
        assert np.linalg.norm(T.dot(x_true) - d) == 0, 'Tx != d'
        assert_scaled_incidence(A)
        data = { 'A': A, 'x_true': x_true, 'w': w, 'b': b, 'T': T, 'd': d,
                 'U': U, 'f': f }
        if V is not None and g is not None:
            data['V'], data['g'] = V, g

        if export:
            scipy.io.savemat(prefix + 'small_graph_OD_dense.mat', data, oned_as='column')

    return data

if __name__ == '__main__':

    import sys

    parser = argparse.ArgumentParser()
    parser.add_argument('--prefix', type=str,
                        help="Prefix where to export files to",
                        default='testtest_')
    parser.add_argument('--nrow', type=int,
                        help='Number of rows in the road grid.', default=5)
    parser.add_argument('--ncol', type=int,
                        help='Number of cols in the road grid.', default=5)
    parser.add_argument('--nodroutes', type=int,
                        help='Number of routes per O-D pair.', default=2)
    parser.add_argument('--nnz_oroutes', type=int,
                        help='Number of non-zero routes per origin.', default=2)
    args = parser.parse_args()
    #  num_rows = 5 if len(sys.argv) <= 1 else int(sys.argv[1])
    #  num_cols = 5 if len(sys.argv) <= 2 else int(sys.argv[2])
    #  num_routes_per_od_pair = 2 if len(sys.argv) <= 3 else int(sys.argv[3])
    #  num_nonzero_routes_per_origin

    export_matrices(args.prefix, args.nrow, args.ncol,
                    nodroutes=args.nodroutes, nnz_oroutes=args.nnz_oroutes)
