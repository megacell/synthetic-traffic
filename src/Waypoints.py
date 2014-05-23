import numpy as np
from matplotlib import pyplot as plt
import numpy.linalg as la

class Waypoints:

    def __init__(self, bbox=[0,0,1,1], n=100):
        self.bbox = bbox    # [x1,y1,x2,y2]
        self.n = n
        self.wp = {}

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
    def uniform_random_bbox(self,pop,bbox,n=None):
        if not n:
            n = self.n
        waypoints = np.array([])
        while waypoints.shape[0] < n:
            if waypoints.size == 0:
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
                if waypoints.size != 0:
                    waypoints = np.vstack((waypoints,x))
                else:
                    waypoints = x
        self.wp['uniform_rand_bbox'] = np.array(waypoints)

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

        waypoints = np.array([])
        while waypoints.shape[0] < n:
            if waypoints.size == 0:
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
                        x = p[i][j] + (np.subtract(p[i][j], p[i][j])) * pos
                        dx = np.random.normal(scale=(w.bbox[2]-w.bbox[0])/tau)
                        dy = np.random.normal(scale=(w.bbox[3]-w.bbox[1])/tau)
                        x = x + np.array([dx,dy])
                        if not self.in_box(x):
                            continue
                        if waypoints.size != 0:
                            waypoints = np.vstack((waypoints,x))
                        else:
                            waypoints = x
        self.wp['gaussian_polyline'] = np.array(waypoints)

    # gaussian sampling around points
    def gaussian_points(self,p,n=None,bounded=True,tau=300):
        if not n:
            n = self.n
        # select point for each of n samples
        waypoints = np.array([])
        while waypoints.shape[0] < n:
            if waypoints.size == 0:
                samples = n
            else:
                samples = 1
            points_ind = np.random.randint(0,len(p),samples)
            for ind in points_ind:
                dx = np.random.normal(scale=(w.bbox[2]-w.bbox[0])/tau)
                dy = np.random.normal(scale=(w.bbox[3]-w.bbox[1])/tau)
                x = p[ind] + np.array([dx,dy])
                if not self.in_box(x):
                    continue
                if waypoints.size != 0:
                    waypoints = np.vstack((waypoints,x))
                else:
                    waypoints = x
        self.wp['gaussian_points'] = np.array(waypoints)

    def show(self):
        colors = 'crmgbyk'

        # draw bounding box
        bbox = self.bbox
        plt.plot((bbox[0],bbox[0]),(bbox[1],bbox[3]),'k')
        plt.plot((bbox[2],bbox[2]),(bbox[1],bbox[3]),'k')
        plt.plot((bbox[0],bbox[2]),(bbox[1],bbox[1]),'k')
        plt.plot((bbox[0],bbox[2]),(bbox[3],bbox[3]),'k')

        # scatter waypoints
        for i,k in enumerate(self.wp):
            print k, self.wp[k].shape
            plt.scatter(self.wp[k][:,0],self.wp[k][:,1],label=k, \
                    color=colors[i],s=2)

        plt.title('Waypoints')
        plt.xlabel('lat')
        plt.ylabel('lon')
        plt.legend(loc='upper right')
        plt.show()

    def save(self,c):
        import pickle
        pickle.dump(self.wp,open('%s/%s' % (c.DATA_DIR,c.WAYPOINTS_FILE),'w'))

if __name__ == "__main__":
    import config as c
    w = Waypoints(bbox=[-118.17,34.0,-117.95,34.2])

    # uniform points
    w.uniform_random(n=1000)
    print "Uniform random waypoints selected"

    # points along major roads
    import pickle
    roads = pickle.load(open('%s/%s' % (c.DATA_DIR,c.ROAD_FILE)))
    w.gaussian_polyline([y.points for (x,y) in roads],n=1000)
    print "Polyline gaussian waypoints selected"

    # points around PEMS sensors
    import csv
    with open('%s/%s' % (c.DATA_DIR,c.SENSOR_FILE)) as csvfile:
        sensor_reader = csv.DictReader(csvfile)
        sensors = [(float(row['Longitude']),float(row['Latitude'])) for row \
                in sensor_reader]
        w.gaussian_points(sensors,n=1000)
    print "Point gaussian waypoints selected"

    # points by population
    import shapefile
    sf = shapefile.Reader("%s/workplace/tier1wgs84" % c.DATA_DIR)
    shapeRecords = sf.shapeRecords()
    areas = [x.record[1] for x in shapeRecords]
    pop20 = [x.record[4] for x in shapeRecords]
    pop35 = [x.record[8] for x in shapeRecords]
    bbox = [x.shape.bbox for x in shapeRecords]
    ind_bbox_filter = [(i,x) for (i,x) in enumerate(bbox) if w.in_box((x[0],x[1])) or w.in_box((x[2],x[3]))]
    ind, bbox_filter = zip(*ind_bbox_filter)
    pop20_filter = [pop20[x] for x in ind] 
    w.uniform_random_bbox(pop20_filter,bbox_filter,n=3000)
    print "Bbox uniform waypoints selected"

    # plot
    w.show()
