from abc import abstractmethod

class TrafficNetwork:
    """
    Bundles road network and traffic network dynamics
    """
    def __init__(self):
        self.G = None
        self.bbox = None

    # checks if point is in bounding box
    def in_box(self,p):
        x1,y1,x2,y2 = self.bbox
        return x1<=p[0]<=x2 and y1<=p[1]<=y2

    @abstractmethod
    def num_links(self):
        """
        Returns the number of links in the network
        :return:
        """
        return NotImplemented

    @abstractmethod
    def get_route_flow(self,i):
        """
        Returns the route flow for route index i
        :param i:
        :return:
        """
        return NotImplemented

    @abstractmethod
    def get_bounding_box(self):
        """
        Returns the bounding box for the network (x1,y1,x2,y2)
        :return:
        """
        return NotImplemented

    @abstractmethod
    def get_heavy_points(self, thresh=None):
        """
        Returns the points to sample along for NL cellpath sensors
        :param thresh:
        :return:
        """
        return NotImplemented

    @abstractmethod
    def get_region_weights(self):
        """
        Returns a list of weight + bounding box tuples, for sampling of NS
        cellpath sensors
        :return:
        """
        return NotImplemented