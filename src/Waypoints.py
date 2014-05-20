import numpy as np
from matplotlib import pyplot as plt
import numpy.linalg as la

class Waypoints:

    def __init__(self, bbox=[0,0,1,1], n=100):
        self.bbox = bbox    # [x1,y1,x2,y2]
        self.n = n
        self.wp = {}

    # transform points from [0,1]^2 to bbox
    def shift_and_scale(self,D):
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
        if not n:
            n = self.n
        pass

    # uniform (random)
    def uniform_random(self,n=None):
        if not n:
            n = self.n
        bbox = self.bbox
        samples = np.random.rand(n,2)
        self.wp['uniform_random'] = self.shift_and_scale(samples)

    # uniform (random) in a polygon
    def uniform_random_polygon(self,n=None):
        if not n:
            n = self.n
        pass

    # gaussian sampling along polyline
    def gaussian_polyline(self,p,n=None,log=False,bounded=True,tau=300):
        if not n:
            n = self.n
        if log:
            dists = np.log([np.square(la.norm(np.array(points)[:-1,:] - \
                    np.array(points)[1:,:])) for points in p])
        else:
            dists = [np.square(la.norm(np.array(points)[:-1,:] - \
                    np.array(points)[1:,:])) for points in p]
        dists_sub = [np.sum(np.square(np.array(points)[:-1,:] - \
                np.array(points)[1:,:]),axis=1) for points in p]
        total = np.sum(dists)
        # select roads for each of n samples
        road_freq = np.random.multinomial(n,dists/total)
        print road_freq
        # for each selected road, select segment for each of n samples
        # TODO get the right samples
        seq_freq = [np.random.multinomial(j,dists_sub[i] / \
                sum(dists_sub[i])) for i,j in enumerate(road_freq)]
        # for each segment, uniformly sample
        locs = np.random.random(n)
        waypoints = None
        for i,freqs in enumerate(seq_freq):
            for j,freq in enumerate(freqs):
                for k in range(freq):
                    pos = np.random.random()
                    x = p[i][j] + (np.subtract(p[i][j], p[i][j])) * pos
                    dx = np.random.normal(scale=(w.bbox[2]-w.bbox[0])/tau)
                    dy = np.random.normal(scale=(w.bbox[3]-w.bbox[1])/tau)
                    x = x + np.array([dx,dy])
                    if not self.in_box(x):
                        continue
                    if waypoints != None:
                        waypoints = np.vstack((waypoints,x))
                    else:
                        waypoints = x
        self.wp['gaussian_polyline'] = np.array(waypoints)


    # gaussian sampling around points
    def gaussian_points(self,p,n=None):
        if not n:
            n = self.n
        pass

    def show(self):
        colors = 'crgbykm'

        # draw bounding box
        bbox = self.bbox
        plt.plot((bbox[0],bbox[0]),(bbox[1],bbox[3]),'k')
        plt.plot((bbox[2],bbox[2]),(bbox[1],bbox[3]),'k')
        plt.plot((bbox[0],bbox[2]),(bbox[1],bbox[1]),'k')
        plt.plot((bbox[0],bbox[2]),(bbox[3],bbox[3]),'k')

        # scatter waypoints
        for i,k in enumerate(self.wp):
            plt.scatter(self.wp[k][:,0],self.wp[k][:,1],label=k,color=colors[i],s=1)

        plt.title('Waypoints')
        plt.xlabel('lat')
        plt.ylabel('lon')
        plt.legend(loc='upper right')
        plt.show()

if __name__ == "__main__":
    w = Waypoints(bbox=[-118.17,34.0,-117.95,34.2])
    w.uniform_random(n=1000)

    import pickle
    import config as c
    roads = pickle.load(open('%s/%s' % (c.DATA_DIR,c.ROAD_FILE)))
    w.gaussian_polyline([y.points for (x,y) in roads],n=1000)

    w.show()
