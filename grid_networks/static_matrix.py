import ipdb

import argparse
import collections
import random

import matplotlib
import networkx as nx
import numpy as np
import scipy.io

import small_graph
import flows

def generate_static_matrix(graph, routes, sensors, flow_portions,
        flow_from_each_node=1.0):
  # All route indices are with respect to _routes_.
  route_indices_by_origin = flows.get_route_indices_by_origin(routes)
    
  b = np.array([graph[u][v]['flow'] for (u,v) in sensors])
  
  xs = []
  mus = []
  phis = []
  num_routes = []
  for node in graph.nodes():
    route_indices_from_node = route_indices_by_origin[node]
    edges_in_route = [collections.Counter(zip(routes[i], routes[i][1:])) for i in \
            route_indices_from_node]
    
    x = np.zeros(shape=len(route_indices_from_node))
    mu = np.zeros(shape=len(route_indices_from_node))
    A = np.zeros(shape=(len(sensors), len(route_indices_from_node)))
    for j in xrange(len(route_indices_from_node)):
      x[j] = flow_portions[route_indices_from_node[j]]
      route = routes[route_indices_from_node[j]]
      mu[j] = sum(1./graph[de[0]][de[1]]['weight'] for de in zip(route,
              route[1:]))
      
      for i in xrange(len(sensors)):
        if sensors[i] in edges_in_route[j]:
          A[i, j] = flow_from_each_node
    
    num_routes.append(len(route_indices_from_node))
    phis.append(A)
    mus.append(mu)
    xs.append(x)

  A,x,mu = np.hstack(phis), np.concatenate(xs), np.concatenate(mus)
  # FIXME need to regenerate b because some routes went over the same link
  # twice, which we aren't counting
  b = A.dot(x)

  return A, x, mu, b, np.array(num_routes)

def generate_static_matrix_OD(graph, routes, sensors, flow_portions,
        flow_from_each_node=1.0):
  # All route indices are with respect to _routes_.
  route_indices_by_OD = flows.get_route_indices_by_OD(routes)
    
  # link flow vector
  b = np.array([graph[u][v]['flow'] for (u,v) in sensors])
  
  # initialize
  xs = []
  mus = []
  phis = []
  num_routes = []

  # build x, mu, A, num_routes (for L1 constraints)
  for origin in graph.nodes():
    for dest in graph.nodes():
      selected_route_indices_by_OD = route_indices_by_OD[origin][dest]
      edges_in_route = [collections.Counter(zip(routes[i], routes[i][1:])) for i in \
              selected_route_indices_by_OD]
      # CAUTION: routes may double-count links for some reason
      
      # initialize
      x = np.zeros(shape=len(selected_route_indices_by_OD))
      mu = np.zeros(shape=len(selected_route_indices_by_OD))
      A = np.zeros(shape=(len(sensors), len(selected_route_indices_by_OD)))

      # skip OD blocks that are all 0
      if flow_from_each_node[origin][dest] == 0:
          continue
      if flow_from_each_node[origin][dest] == []:
          continue

      # build A, x, mu block by block (1 origin)
      for j in xrange(len(selected_route_indices_by_OD)):
        x[j] = flow_portions[selected_route_indices_by_OD[j]]
        route = routes[selected_route_indices_by_OD[j]]
        # TODO what is mu?
        mu[j] = sum(1./graph[u][v]['weight'] for (u,v) in zip(route,
                route[1:]))
        
        for i in xrange(len(sensors)):
          if sensors[i] in edges_in_route[j]:
            A[i, j] = flow_from_each_node[origin][dest]
      
      num_routes.append(len(selected_route_indices_by_OD))
      phis.append(A)
      mus.append(mu)
      xs.append(x)

  A,x,mu = np.hstack(phis), np.concatenate(xs), np.concatenate(mus)
  # FIXME need to regenerate b because some routes went over the same link
  # twice, which we aren't counting
  b = A.dot(x)

  return A, x, mu, b, np.array(num_routes)

          
def generate_random_matrix(graph, routes, sensors, flow_portions,
                  flow_from_each_node=1.0):
  # All route indices are with respect to _routes_.
  route_indices_by_origin = flows.get_route_indices_by_origin(routes)
    
#  b = np.array([graph[u][v]['flow'] for (u,v) in sensors])
  
  xs = []
  mus = []
  num_routes = []
  for node in graph.nodes():
    route_indices_from_node = route_indices_by_origin[node]

    x = np.zeros(shape=len(route_indices_from_node))
    mu = np.zeros(shape=len(route_indices_from_node))
    for j in xrange(len(route_indices_from_node)):
      x[j] = flow_portions[route_indices_from_node[j]]
      route = routes[route_indices_from_node[j]]
      mu[j] = sum(1./graph[de[0]][de[1]]['weight'] for de in zip(route,
              route[1:]))
    num_routes.append(len(route_indices_from_node))
    mus.append(mu)
    xs.append(x)
  
  x = np.concatenate(xs)
  A = np.random.normal(size=(len(sensors), len(routes)))  
  b = np.dot(A, x)

  return A, np.concatenate(xs), np.concatenate(mus), b, \
          np.array(num_routes)

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
  graph, routes, sensors = small_graph.generate_small_graph(num_cols=num_cols,
          num_rows=num_rows, num_routes_per_od_pair=num_routes_per_od_pair)
  # (O,D),R,x
  (flow_portions,flow_portions_OD,flow_OD) = flows.annotate_with_flows(graph, 
          routes, 1.0, num_nonzero_routes_per_o)
  
  # static matrix considering origin flows
  A, x_true, mu, b, num_routes = generate_static_matrix(graph, routes,
          sensors, flow_portions)
  assert_scaled_incidence(A)
  scipy.io.savemat(prefix + 'small_graph.mat', {'A': A, 'x_true': x_true,
                'w': mu, 'b': b, 'block_sizes': num_routes}, oned_as='column')

  # static matrix considering origin-destination flows
  A, x_true, mu, b, num_routes = generate_static_matrix_OD(graph, routes,
          sensors, flow_portions_OD, flow_from_each_node=flow_OD)
  assert_scaled_incidence(A)
  scipy.io.savemat(prefix + 'small_graph_OD.mat', {'A': A, 'x_true': x_true,
        'w': mu, 'b': b, 'block_sizes': num_routes}, oned_as='column')

  # random matrix 'considering origin flows'
  A, x_true, mu, b, num_routes = generate_random_matrix(graph, routes,
          sensors, flow_portions)
  scipy.io.savemat(prefix + 'small_graph_random.mat', {'A': A, 'x_true': x_true,
        'w': mu, 'b': b, 'block_sizes': num_routes}, oned_as='column')

  # same graph but dense OD blocks (CANNOT be used for comparison with above)
  (flow_portions,flow_portions_OD,flow_OD) = \
      flows.annotate_with_flows_dense_blocks(graph, routes, 1.0,
                                             overall_sparsity=0.1)

  # static matrix considering origin-destination flows
  A, x_true, mu, b, num_routes = generate_static_matrix_OD(graph, routes,
          sensors, flow_portions_OD, flow_from_each_node=flow_OD)
  assert_scaled_incidence(A)
  scipy.io.savemat(prefix + 'small_graph_OD_dense.mat', {'A': A,
        'x_true': x_true, 'w': mu, 'b': b, 'block_sizes': num_routes},
                   oned_as='column')
  ipdb.set_trace()

  
if __name__ == '__main__':
  import sys
  
  parser = argparse.ArgumentParser()
  parser.add_argument('--prefix', type=str, help="Prefix where to export files to", default='testtest_')
  parser.add_argument('--num_rows', type=int, help='Number of rows in the road grid.', default=5)
  parser.add_argument('--num_cols', type=int, help='Number of cols in the road grid.', default=5)
  parser.add_argument('--num_routes_per_od', type=int, help='Number of routes per O-D pair.', default=2)
  parser.add_argument('--num_nonzero_routes_per_o', type=int, help='Number of non-zero routes per origin.', default=2)
  args = parser.parse_args()
#  num_rows = 5 if len(sys.argv) <= 1 else int(sys.argv[1])
#  num_cols = 5 if len(sys.argv) <= 2 else int(sys.argv[2])
#  num_routes_per_od_pair = 2 if len(sys.argv) <= 3 else int(sys.argv[3])
#  num_nonzero_routes_per_origin 

  export_matrices(args.prefix, args.num_rows, args.num_cols, args.num_routes_per_od, args.num_nonzero_routes_per_o)

