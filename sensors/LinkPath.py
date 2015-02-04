import ipdb
import random

from synth_utils import to_np, to_sp, simplex as simplex_base

__author__ = 'cathywu'

class LinkPath:
    def __init__(self, graph, N=10):
        self.N = N
        self.sample_linkpath(graph)

    def sample_linkpath(self, graph):
        if graph.__class__.__name__ == 'Graph':
            self.lp = random.sample(graph.G.links.keys(),self.N)
        elif graph.__class__.__name__ == 'GridNetwork':
            self.lp = random.sample(graph.G.edges(),self.N)

    def update_trajs(self,graph):
        if graph.__class__.__name__ == 'Graph':
            self._get_trajs_UE(graph)
            self._update_flows_UE(graph)
        elif graph.__class__.__name__ == 'GridNetwork':
            self._get_trajs_grid(graph)
            self._update_flows_grid(graph)

    # FIXME unify _get_trajs_*
    def _get_trajs_UE(self, graph):
        rs = graph.G.paths
        path_lps = [(r,[e.repr() for e in rs[r].links if e.repr() in self.lp]) \
                    for r in rs.keys()]
        lps = {}
        for value,key in path_lps:
            lps.setdefault(tuple(key), []).append(value)
        if () in lps:
            del lps[()]
        self.path_lps, self.trajs = path_lps, lps
    def _get_trajs_grid(self, graph, r_ids=None):
        rs = graph.routes
        if not r_ids:
            r_ids = xrange(len(rs))
        path_lps = [[e for e in zip(rs[r]['path'],rs[r]['path'][1:]) \
                     if e in self.lp] for r in r_ids]
        lps = {}
        for value,key in enumerate(path_lps):
            lps.setdefault(tuple(key), []).append(value)
        if () in lps:
            del lps[()]
        self.path_lps, self.trajs = path_lps, lps

    # FIXME unify
    def _update_flows_UE(self, graph):
        # FIXME
        self.flows = [sum([graph.G.paths[i].flow for i in paths]) for \
                         paths in self.trajs.values()]
    def _update_flows_grid(self, graph):
        self.flows = [sum([graph.get_route_flow(i) for i in paths]) for \
                         paths in self.trajs.values()]

    # FIXME unify
    def simplex(self,graph):
        if graph.__class__.__name__ == 'Graph':
            return self._simplex_UE(graph)
        elif graph.__class__.__name__ == 'GridNetwork':
            return self._simplex_grid(graph)
    def _simplex_UE(self,graph):
        """Build simplex constraints from lp flows
        """
        from cvxopt import matrix, spmatrix
        n = len(self.trajs)
        m = len(graph.G.paths)
        if n == 0:
            return None, None
        I, J, r = [], [], matrix(0.0, (n,1))
        for i, path_ids in enumerate(self.trajs.itervalues()):
            r[i] = self.flows[i]
            for id in path_ids:
                I.append(i)
                J.append(graph.G.indpaths[id])
        V = to_sp(spmatrix(1.0, I, J, (n, m)))
        r = to_np(r)
        return V, r
    def _simplex_grid(self,graph):
        return simplex_base(len(graph.routes),self.trajs,self.flows)

