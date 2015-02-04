from abc import abstractmethod

class TrafficNetwork:

    def __init__(self):
        self.graph = None
        self.bbox = None

    # checks if point is in bounding box
    def in_box(self,p):
        if p[0] >= self.bbox[0] and p[0] <= self.bbox[2] and \
                        p[1] >= self.bbox[1] and p[1] <= self.bbox[3]:
            return True
        return False

    @abstractmethod
    def get_links(self):
        return NotImplemented

    @abstractmethod
    def get_routes(self):
        return NotImplemented

    @abstractmethod
    def get_route_flow(self,i):
        return NotImplemented

    @abstractmethod
    def get_bounding_box(self):
        return NotImplemented

    @abstractmethod
    def get_heavy_points(self, thresh=None):
        return NotImplemented

    @abstractmethod
    def get_region_weights(self):
        return NotImplemented