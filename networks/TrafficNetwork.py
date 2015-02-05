from abc import abstractmethod

class TrafficNetwork:
    """
    Bundles road network and traffic network dynamics
    """
    def __init__(self):
        self.graph = None
        self.bbox = None

    # checks if point is in bounding box
    def in_box(self,p):
        x1,y1,x2,y2 = self.bbox
        return x1<=p[0]<=x2 and y1<=p[1]<=y2

    @abstractmethod
    def num_links(self):
        return NotImplemented

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