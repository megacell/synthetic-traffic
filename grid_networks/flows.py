import ipdb

import collections
import random

import matplotlib
import networkx as nx
import numpy.random

def annotate_with_flows(grid, flow_from_each_node=1.0, num_nonzero_routes=2):
    '''Generate traffic from each origin onto some small fraction of its routes, \
            and compute the amount of flow at each edge.'''

    # collect routes by origin or by OD pair
    # Note: All route indices are with respect to _routes_.
    route_indices_by_origin = grid.get_route_indices_by_origin()
    route_indices_by_OD = grid.get_route_indices_by_OD()

    flow_portions = [0] * len(grid.routes) # from origin
    flow_portions_OD = [0] * len(grid.routes) # from origin to destination
    flow_OD = grid.new_dict_OD()

    # initialize the flows, in case a node is not in the interior of any route
    for n in grid.G.nodes():
      grid.G.node[n]['2nd_flow'] = {}

    # initialize all flows on edges to 0
    for (u,v) in grid.G.edges():
        grid.G.edge[u][v]['flow'] = 0

    # sample routes and compute aggregate first-order, second-order info on
    # origins and origin-destination pairs

    for origin in grid.G.nodes():
        # consider all routes out of origin
        route_indices_from_origin = route_indices_by_origin[origin]
        #    num_nonzero_routes = max(1, int(sparsity * len(route_indices_from_origin)))
        # select routes with non-zero flow
        selected_route_indices = sorted(random.sample(route_indices_from_origin,
                                                      num_nonzero_routes))
        # probability prior (uniform dirichlet) on non-zero flow routes
        selected_route_weights = numpy.random.dirichlet([1] * num_nonzero_routes,
                                                        1)[0]

        grid.update_flows(flow_portions, flow_from_each_node,
                           selected_route_indices,selected_route_weights)

        # normalize flows for each OD pair
        for dest in grid.G.nodes():
            # normalize weights per OD pair
            selected_route_indices_OD = route_indices_by_OD[origin][dest]
            total = sum(flow_portions[i] for i in selected_route_indices_OD)
            flow_OD[origin][dest] = total
            if total == 0:
                continue
            for i in selected_route_indices_OD:
                flow_portions_OD[i] = flow_portions[i]/total

    # return (flow_portions, flow_portions_OD,flow_OD) # used to generate real alpha
    return grid.G, grid.routes

def annotate_with_flows_dense_blocks(grid, flow_from_each_node=1.0, overall_sparsity=0.1):
    '''Generate traffic from each origin onto some small fraction of its routes, \
            and compute the amount of flow at each edge.'''

    # collect routes by origin or by OD pair
    # Note: All route indices are with respect to _routes_.
    route_indices_by_origin = grid.get_route_indices_by_origin()
    route_indices_by_OD = grid.get_route_indices_by_OD()

    flow_portions = [0] * len(grid.routes) # from origin
    flow_portions_OD = [0] * len(grid.routes) # from origin to destination
    flow_OD = grid.new_dict_OD()
    OD_pairs = grid.get_OD_pairs()

    # initialize the flows, in case a node is not in the interior of any route
    for n in grid.G.nodes():
        grid.G.node[n]['2nd_flow'] = {}

    # initialize all flows on edges to 0
    for (u,v) in grid.G.edges():
        grid.G.edge[u][v]['flow'] = 0

    # sample OD pairs
    selected_OD_pairs = random.sample(OD_pairs, int(len(OD_pairs) * overall_sparsity))
    for origin,dest in selected_OD_pairs:
        flow_OD[origin][dest] = flow_from_each_node
        selected_route_indices = route_indices_by_OD[origin][dest]
        num_routes = len(selected_route_indices)
        selected_route_weights = numpy.random.dirichlet([1] * num_routes, 1)[0]

        grid.update_flows(flow_portions_OD, flow_from_each_node,
                           selected_route_indices,selected_route_weights)

    # return (flow_portions, flow_portions_OD,flow_OD) # used to generate real alpha
    return grid.G, grid.routes

if __name__ == '__main__':
    from GridNetwork import GridNetwork

    grid = GridNetwork()

    print "Graph: "
    print grid.G
    print "Routes: "
    # print grid.routes
    print len(grid.routes)
    print "Sensors: "
    print grid.sensors

    annotate_with_flows(grid)

    pos = nx.get_node_attributes(grid.G, 'pos')
    nx.draw(grid.G,pos)

    edge_labels = collections.defaultdict(float)
    for u, v, d in grid.G.edges(data=True):
      if u > v:
        u, v = v, u
      edge_labels[(u, v)] += d['flow'] if 'flow' in d else 0
    #  edge_labels=dict([((u,v,),d['flow'] if 'flow' in d else 0)
    #                    for u,v,d in graph.edges(data=True)])
    for u, v, d in grid.G.edges(data=True):
      if u > v:
        u, v = v, u
      edge_labels[(u, v)] = round(edge_labels[(u, v)], 4)
    nx.draw_networkx_edge_labels(grid.G, pos, edge_labels=edge_labels)

    matplotlib.pyplot.show()
