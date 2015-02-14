#!/usr/bin/env python

"""
CellPath object maintains various sampled types of cellular sensors,
cellpath trajectory extraction from routes, and cellpath flow computation from
TrafficNetwork object
"""

import ipdb
import logging

import numpy as np
from matplotlib import pyplot as plt
import numpy.linalg as la

from synth_utils import matrix, simplex as simplex_base, to_np, to_sp

__author__ = 'cathywu'

class CellPath:

    def __init__(self, bbox=(0,0,1,1), n=0, TN=None, NB=None, NL=None,
                 NS=None, freq=10, thresh=5, scale=None):
        self.bbox = bbox    # [x1,y1,x2,y2]
        self.n = n
        self.cp = {}
        self.NB, self.NL, self.NS = NB, NL, NS
        self.freq, self.thresh = freq, thresh # GridNetwork only
        self.scale = scale # EquilibriumNetwork only
        if TN is not None:
            self.sample_from_TN(TN)

    def sample_from_TN(self, TN):
        if TN.__class__.__name__ == 'EquilibriumNetwork':
            self._sample_from_TN_eq(TN)
        elif TN.__class__.__name__ == 'GridNetwork' or \
                        TN.__class__.__name__ == 'LosAngelesOSMNetwork':
            self._sample_from_TN_grid(TN)
        else:
            return NotImplemented

    def _sample_from_TN_eq(self, TN, margin=0.05):
        """Sample waypoints on TN

        Parameters:
        -----------
        TN: Graph object
        N0: number of background samples
        N1: number of samples on links
        regions: list of regions, regions[k] = (geometry, N_region)
        margin: % size of margin around the TN
        """
        from networks.wardrop.Waypoints import BoundingBox
        xs = [p[0] for p in TN.G.nodes_position.values()]
        ys = [p[1] for p in TN.G.nodes_position.values()]
        min_x, max_x, min_y, max_y = min(xs), max(xs), min(ys), max(ys)
        w, h = max_x-min_x, max_y-min_y
        x1, x2, y1, y2 = min_x - w*margin, max_x + w*margin, min_y - h*margin, \
                         max_y + h*margin
        WP = BoundingBox((x1, y1, x2, y2))
        WP.populate(self.NB)
        total_length, lines = 0, []
        for link in TN.G.links.values():
            xs, ys = TN.G.nodes_position[link.startnode]
            xt, yt = TN.G.nodes_position[link.endnode]
            length = np.linalg.norm([xs-xt, ys-yt])
            total_length += length
            lines.append([(xs,ys,xt,yt), length])
        weights = [line[1]/total_length for line in lines]
        Ns = np.random.multinomial(self.NL, weights, size=1)[0]
        for k,line in enumerate(lines): WP.add_line(line[0], Ns[k], self.scale)
        weights, bboxes = TN.get_region_weights()
        weights_total = sum(weights)
        for w,b in zip(weights,bboxes):
            WP.add_rectangle(b, self.NS*w/weights_total)

        self.cp = {'AllTypes' : matrix(WP.wp.values())}
    def _sample_from_TN_grid(self,TN):
        self.bbox = TN.get_bounding_box() # x1, y1, x2, y2
        # uniformly sample points in bbox
        if self.NB is not None and self.NB > 0:
            self.uniform_random(n=self.NB)
        # sample points along heavy edges (main roads)
        if self.NL is not None and self.NL > 0:
            heavy_points = TN.get_heavy_points(thresh=self.thresh)
            if len(heavy_points) > 1:
                self.gaussian_polyline(heavy_points,n=self.NL,tau=30)
        if self.NS is not None and self.NS > 0:
            weights, bboxes = TN.get_region_weights()
            if len(weights) > 0:
                self.uniform_random_bbox(weights,bboxes,n=self.NS)

    def update_trajs(self,TN):
        if TN.__class__.__name__ == 'EquilibriumNetwork':
            self._get_trajs_eq(TN,self.freq)
            # self._update_flows_eq(TN)
        elif TN.__class__.__name__ == 'GridNetwork':
            self._get_trajs_grid(TN,self.freq)
            self._update_flows_grid(TN)
        else:
            return NotImplemented

    # transform points from [0,1]^2 to bbox
    def shift_and_scale(self,D,bbox=None):
        if not bbox:
            bbox = self.bbox
        x_new = bbox[0] + D[:,0] * (bbox[2]-bbox[0])
        y_new = bbox[1] + D[:,1] * (bbox[3]-bbox[1])
        return np.vstack((x_new, y_new)).T

    # checks if point is in bounding box
    def in_box(self,p):
        if p[0] >= self.bbox[0] and p[0] <= self.bbox[2] and \
                p[1] >= self.bbox[1] and p[1] <= self.bbox[3]:
            return True
        return False

    # uniform (deterministic)
    def uniform(self,n):
        if n is None:
            n = self.n
        elif n == 0:
            return
        pass

    # uniform (random)
    def uniform_random(self,n=None):
        if n is None:
            n = self.n
        elif n == 0:
            return
        bbox = self.bbox
        samples = np.random.rand(n,2)
        self.cp['uniform_random'] = matrix(self.shift_and_scale(samples))

    # uniform (random) in a polygon
    def uniform_random_bbox(self,pop,bbox,n=None):
        if n is None:
            n = self.n
        elif n == 0:
            return
        cells = np.array([])
        while cells.shape[0] < n:
            if cells.size == 0:
                samples = n
            else:
                samples = 1
            # select bbox for each of n samples
            bbox_freq = np.random.multinomial(samples,pop/np.sum(pop))
            # for each bbox, uniformly sample
            for i,freq in enumerate(bbox_freq):
                if freq == 0:
                    continue
                x = self.shift_and_scale(np.random.rand(freq,2),bbox=bbox[i])
                x = np.array([p for p in x if self.in_box(p)])
                if x.size == 0:
                    continue
                if cells.size != 0:
                    cells = np.vstack((cells,x))
                else:
                    cells = x
        self.cp['uniform_rand_bbox'] = matrix(cells)

    # gaussian sampling along polyline
    def gaussian_polyline(self,p,n=None,log=False,bounded=True,tau=300):
        if n is None:
            n = self.n
        elif n == 0:
            return
        if log:
            dists = np.log([np.square(la.norm(np.array(points)[:-1,:] - \
                    np.array(points)[1:,:])) for points in p])
        else:
            dists = [np.square(la.norm(np.array(points)[:-1,:] - \
                    np.array(points)[1:,:])) for points in p]
        dists_sub = [np.sum(np.square(np.array(points)[:-1,:] - \
                np.array(points)[1:,:]),axis=1) for points in p]
        total = np.sum(dists)

        cells = np.array([])
        while cells.shape[0] < n:
            if cells.size == 0:
                samples = n
            else:
                samples = 1
            # select polyline for each of n samples
            road_freq = np.random.multinomial(samples,dists/total)
            # for each selected polyline, select segment for each of n samples
            seq_freq = [np.random.multinomial(j,dists_sub[i] / \
                    sum(dists_sub[i])) for i,j in enumerate(road_freq)]
            # for each segment, uniformly sample
            locs = np.random.random(samples)
            for i,freqs in enumerate(seq_freq):
                for j,freq in enumerate(freqs):
                    if freq == 0:
                        continue
                    for k in range(freq):
                        pos = np.random.random()
                        x = p[i][j] + (np.subtract(p[i][j], p[i][j+1])) * pos
                        dx = np.random.normal(scale=(self.bbox[2]-self.bbox[0])/tau)
                        dy = np.random.normal(scale=(self.bbox[3]-self.bbox[1])/tau)
                        x = x + np.array([dx,dy])
                        if not self.in_box(x):
                            continue
                        if cells.size != 0:
                            cells = np.vstack((cells,x))
                        else:
                            cells = x
        self.cp['gaussian_polyline'] = matrix(cells)

    # gaussian sampling around points
    def gaussian_points(self,p,n=None,bounded=True,tau=300):
        if n is None:
            n = self.n
        elif n == 0:
            return
        # select point for each of n samples
        cells = np.array([])
        while cells.shape[0] < n:
            if cells.size == 0:
                samples = n
            else:
                samples = 1
            points_ind = np.random.randint(0,len(p),samples)
            for ind in points_ind:
                dx = np.random.normal(scale=(self.bbox[2]-self.bbox[0])/tau)
                dy = np.random.normal(scale=(self.bbox[3]-self.bbox[1])/tau)
                x = p[ind] + np.array([dx,dy])
                if not self.in_box(x):
                    continue
                if cells.size != 0:
                    cells = np.vstack((cells,x))
                else:
                    cells = x
        self.cp['gaussian_points'] = matrix(cells)

    def draw(self):
        colors = 'rbmgcyk'

        # draw bounding box
        bbox = self.bbox
        plt.plot((bbox[0],bbox[0]),(bbox[1],bbox[3]),'k')
        plt.plot((bbox[2],bbox[2]),(bbox[1],bbox[3]),'k')
        plt.plot((bbox[0],bbox[2]),(bbox[1],bbox[1]),'k')
        plt.plot((bbox[0],bbox[2]),(bbox[3],bbox[3]),'k')

        # scatter cells
        for i,k in enumerate(self.cp):
            print k, self.cp[k].shape
            plt.scatter(self.cp[k][:,0],self.cp[k][:,1],label=k, \
                    color=colors[i],s=2)

        plt.title('CellPaths')
        plt.xlabel('lat')
        plt.ylabel('lon')
        plt.legend(loc='upper right')

    def show(self):
        self.draw()
        plt.show()

    def save(self,c):
        import pickle
        total = sum([len(v) for (k,v) in self.cp.iteritems()])
        pickle.dump(self.cp,open('%s/%s' % (c.DATA_DIR,
            c.WAYPOINTS_FILE % total),'w'))

    def load(self,c):
        import pickle
        return pickle.load(open('%s/%s' % (c.DATA_DIR,c.WAYPOINTS_FILE)))

    def closest_to_point(self, point, fast=False):
        """Find closest cell to a point (x,y)
        Note: fast is only available in Rectangle class"""
        id = None
        min_dist = np.inf
        # FIXME UPDATE fast method not used here
        if fast:
            x1,y1,x2,y2 = self.geometry
            res = self.partition[0]
            w, h = (x2-x1)/res[0], (y2-y1)/res[1]
            i = min(int(np.floor((point[0]-x1)/w)), res[0]-1)
            j = min(int(np.floor((point[1]-y1)/h)), res[1]-1)
            ids = self.partition[1][(i,j)]
        else:
            ids = self.cp.keys() #explore all ids
        for id in ids:
            x, y = self.cp[id][:,0], self.cp[id][:,1]
            d = np.linalg.norm([point[0]-x, point[1]-y], axis=0)
            d_min, i_min = np.min(d), np.argmin(d)
            if d_min < min_dist: min_dist, id = d_min, (id,i_min)

        return id


    def closest_to_line(self, directed_line, n, fast=False):
        """Find list of closest cells to a directed_line

        Parameters:
        ----------
        directed_line: (x1,y1,x2,y2)
        n: number of points to take on the line
        """
        x1,y1,x2,y2 = directed_line
        interp_x = np.linspace(x1,x2,num=n)
        interp_y = np.linspace(y1,y2,num=n)
        ids = [self.closest_to_point((x,y), fast) for (x,y) in zip(interp_x,interp_y)]
        ids_deduped = [ids[0]]
        ids_deduped.extend([y for (x,y) in zip(ids,ids[1:]) if x!=y])
        return ids_deduped


    def closest_to_polyline(self, polyline, n, fast=False):
        """Find list of closest cells to a directed polyline

        Parameters:
        ----------
        polyline: list of directed lines [(x1,y1,x2,y2)]
        n: number of points to take on each line of the polyline
        """
        ids = [self.closest_to_line(line, n, fast) for line in polyline]
        ids = [item for sublist in ids for item in sublist]
        ids_deduped = [ids[0]]
        ids_deduped.extend([y for (x,y) in zip(ids,ids[1:]) if x!=y])
        return ids_deduped


    def closest_to_path(self, graph, path, n, fast=False):
        """Find list of closest cells to a path in the TN

        Parameters:
        ----------
        TN: Graph object
        path: sequence of nodes in path
        n: number of points to take on each link of the path
        """
        polyline = []
        if graph.__class__.__name__ == 'Graph':
            path_id = path
            for link in graph.paths[path_id].links:
                x1, y1 = graph.nodes_position[link.startnode]
                x2, y2 = graph.nodes_position[link.endnode]
                polyline.append((x1,y1,x2,y2))
        elif graph.__class__.__name__ == 'DiGraph':
            pos = [graph.node[x]['pos'] for x in path]
            for (i,x) in enumerate(pos[:-1]):
                x1, y1 = x
                x2, y2 = pos[i+1]
                polyline.append((x1,y1,x2,y2))
        else:
            return NotImplemented
        return self.closest_to_polyline(polyline, n, fast)

    def _get_trajs_grid(self, TN, n, r_ids=None, fast=False, tol=1e-3):
        """Compute cellpath trajectories and returns {path_id: ids}, [(traj, path_list, flow)]

        Parameters:
        ----------
        TN: Graph object with path flows in it
        n: number of points to take on each link of paths
        fast: if True do fast computation
        tol:

        Return value:
        ------------
        path_cps: dictionary of paths with >tol flow with cp trajectory associated {path_id: ids}
        trajs: list of cells trajectories with paths along this trajectory [(traj, path_list, flow)]
        """
        if not r_ids:
            r_ids = xrange(len(TN.routes))
        path_cps = [self.closest_to_path(TN.G, TN.routes[r]['path'], n,
                                         fast=fast) for r in r_ids]
        cps = {}
        for value,key in enumerate(path_cps):
            cps.setdefault(tuple(key), []).append(value)
        self.path_cps, self.trajs = path_cps, cps

    def _update_flows_grid(self, TN):
        self.flows = [sum([TN.routes[i]['flow'] for i in paths]) for \
                         paths in self.trajs.values()]

    def _get_trajs_eq(self, TN, n, r_ids=None, fast=False, tol=1e-3):
        """Compute CellPath trajectories and returns {path_id: ids}, [(traj, path_list, flow)]

        Parameters:
        ----------
        TN: Graph object with path flows in it
        n: number of points to take on each link of paths
        fast: if True do fast computation
        tol: consider only paths for which flow on it is more than tol

        Return value:
        ------------
        path_cps: dictionary of all the paths with flow>tol and with a list of closest waypoints to it
        or associated cp trajectory {path_id: ids}
        trajs: list of waypoint trajectories with paths along this trajectory [(traj, path_list, flow)]
        """
        if self.cp == None or len(self.cp) == 0:
            return None, None
        path_cps, k = {}, 0
        for path_id, path in TN.G.paths.items():
            # if path.flow > tol:
            k += 1
            if k%10 == 0: logging.debug('Number of paths processed: %s' % k)
            ids = self.closest_to_path(TN.G, path_id, n, fast)

            path_cps[path_id] = ids
        cps_list, paths_list, flows_list = [], [], []
        for path_id, cps in path_cps.items():
            try:
                index = cps_list.index(cps) # find the index of cps in cps_list
                paths_list[index].append(path_id)
                flows_list[index] += TN.G.paths[path_id].flow
            except ValueError: # cps not in cps_list
                cps_list.append(cps)
                paths_list.append([path_id])
                flows_list.append(TN.G.paths[path_id].flow)
        self.path_cps, self.trajs = path_cps, zip(cps_list, paths_list, flows_list)

    def _update_flows_eq(self, TN):
        # TODO I AM HERE
        self.flows = [sum([TN.G.routes[i]['flow'] for i in paths]) for \
                         paths in self.trajs.values()]

    def simplex(self,TN):
        if TN.__class__.__name__ == 'EquilibriumNetwork':
            return self._simplex_eq(TN)
        elif TN.__class__.__name__ == 'GridNetwork':
            return self._simplex_grid(TN)
        return NotImplemented

    def _simplex_eq(self,TN):
        """Build simplex constraints from lp flows
        """
        from cvxopt import matrix as mat, spmatrix
        n = len(self.trajs)
        I, J, r, i = [], [], mat(0.0, (n,1)), 0
        for wp_traj, path_ids, flow in self.trajs:
            r[i] = flow
            for id in path_ids:
                I.append(i)
                J.append(TN.G.indpaths[id])
            i += 1
        U = to_sp(spmatrix(1.0, I, J, (n, TN.G.numpaths)))
        r = to_np(r)
        return U, r

    def _simplex_grid(self,TN):
        return simplex_base(len(TN.routes),self.trajs,self.flows)

if __name__ == "__main__":
    import unittest
    from tests.test_cellpath import TestCellPath
    unittest.main()
