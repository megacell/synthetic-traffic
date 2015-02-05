
from TrafficNetwork import TrafficNetwork
import config as c

class LosAngelesOSMNetwork(TrafficNetwork):
    def __init__(self):
        self.bbox = self.get_bounding_box()

    def get_bounding_box(self):
        # Official bounding box: -118.328299, 33.984601, -117.68132, 34.255881
        # cp = CellPaths(bbox=[-118.328299, 33.984601, -117.68132, 34.255881])
        # Use larger bounding box
        return [-118.373284700001, 33.9309546999998,-117.6511386, 34.2584316999999]

    def get_heavy_points(self, thresh=None):
        # points along major roads
        import pickle
        roads = pickle.load(open('%s/%s' % (c.DATA_DIR,c.ROAD_FILE)))
        heavy_points = [y.points for (x,y) in roads]
        return heavy_points

    def get_region_weights(self):
        # points by population
        import shapefile
        sf = shapefile.Reader("%s/workplace/tier1wgs84" % c.DATA_DIR)
        shapeRecords = sf.shapeRecords()
        TAZ_IDs = [x.record[3] for x in shapeRecords]
        areas = [x.record[1] for x in shapeRecords]
        pop20 = [x.record[4] for x in shapeRecords]
        emp20 = [x.record[6] for x in shapeRecords]
        pop35 = [x.record[8] for x in shapeRecords]
        bbox = [x.shape.bbox for x in shapeRecords]
        ind_bbox_filter = [(i,x) for (i,x) in enumerate(bbox) if \
                           self.in_box((x[0],x[1])) or self.in_box((x[2],x[3]))]
        ind, bbox_filter = zip(*ind_bbox_filter)
        ppl20_filter = [emp20[x] for x in ind]
        return ppl20_filter,bbox_filter

if __name__ == "__main__":
    rs = [0.25, 0.5, 0.75, 1, 1.5, 2, 2.5, 3, 3.5, 4, 6, 8]
    import config as c
    from sensors.CellPath import CellPath

    for r in rs:
        LA = LosAngelesOSMNetwork()
        cp = CellPath(NB=150*r,NL=50*r,NS=800*r,TN=LA)
        # cp.save(c)
    # plot
    cp.show()
