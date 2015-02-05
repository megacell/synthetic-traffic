import ipdb
import random

from synth_utils import to_np, to_sp, simplex as simplex_base

__author__ = 'cathywu'

class LinkPath:
    def __init__(self, TN, N=10):
        self.N = N
        self.sample_linkpath(TN)

    def sample_linkpath(self, TN):
        if TN.__class__.__name__ == 'EquilibriumNetwork':
            links = TN.G.links.keys()
        elif TN.__class__.__name__ == 'GridNetwork':
            links = TN.G.edges()
        else:
            return NotImplemented
        self.lp = random.sample(links,min(self.N,len(links)))

    def update_trajs(self,TN):
        if TN.__class__.__name__ == 'EquilibriumNetwork':
            self._get_trajs_eq(TN)
            self._update_flows_eq(TN)
        elif TN.__class__.__name__ == 'GridNetwork':
            self._get_trajs_grid(TN)
            self._update_flows_grid(TN)
        else:
            return NotImplemented
    # FIXME unify _get_trajs_*
    def _get_trajs_eq(self, TN):
        rs = TN.G.paths
        path_lps = [(r,[e.repr() for e in rs[r].links if e.repr() in self.lp]) \
                    for r in rs.keys()]
        lps = {}
        for value,key in path_lps:
            lps.setdefault(tuple(key), []).append(value)
        if () in lps:
            del lps[()]
        self.path_lps, self.trajs = path_lps, lps
    def _get_trajs_grid(self, TN, r_ids=None):
        rs = TN.routes
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
    def _update_flows_eq(self, TN):
        # FIXME
        self.flows = [sum([TN.G.paths[i].flow for i in paths]) for \
                         paths in self.trajs.values()]
    def _update_flows_grid(self, TN):
        self.flows = [sum([TN.get_route_flow(i) for i in paths]) for \
                         paths in self.trajs.values()]

    # FIXME unify
    def simplex(self,TN):
        if TN.__class__.__name__ == 'EquilibriumNetwork':
            return self._simplex_eq(TN)
        elif TN.__class__.__name__ == 'GridNetwork':
            return self._simplex_grid(TN)
        return NotImplemented
    def _simplex_eq(self,TN):
        """Build simplex constraints from lp flows
        """
        from cvxopt import matrix, spmatrix
        n = len(self.trajs)
        m = len(TN.G.paths)
        if n == 0:
            return None, None
        I, J, r = [], [], matrix(0.0, (n,1))
        for i, path_ids in enumerate(self.trajs.itervalues()):
            r[i] = self.flows[i]
            for id in path_ids:
                I.append(i)
                J.append(TN.G.indpaths[id])
        V = to_sp(spmatrix(1.0, I, J, (n, m)))
        r = to_np(r)
        return V, r
    def _simplex_grid(self,TN):
        return simplex_base(len(TN.routes),self.trajs,self.flows)

