from django.contrib.gis.geos import LineString, MultiLineString, Point

class Trajectory:
    def __init__(self, id_sequence, od_taz, geometry_map, length_cache):
        self._id_sequence = id_sequence
        self._id_set = set(id_sequence)
        self.od_taz = od_taz
        self._length = -1
        self._geometry_map = geometry_map
        self._start_point = self.get_start_point()
        self._end_point = self.get_end_point()
        self._length_cache = length_cache

    def _trajectory_length(self, sequence):
        total = 0
        for i in sequence:
            total+= self._length_cache[i]
        return total

    def length(self):
        if self._length < 0:
            self._length = self._trajectory_length(self._id_sequence)
        return self._length

    def match_percent(self, other):
        a = (self._id_set)
        b = (other._id_set)
        c = a.intersection(b)
        return self._trajectory_length(c) / max(self.length(), other.length())

    def convert_to_MultiLineString(self):
        lines = [self._geometry_map[id] for id in self._id_sequence]
        multiline = MultiLineString(lines)
        multiline.set_srid(lines[0].get_srid())
        return multiline

    def get_start_point(self):
        g = self._geometry_map[self._id_sequence[0]]
        p = Point(g[0])
        p.set_srid(g.get_srid())
        return p

    def get_end_point(self):
        g = self._geometry_map[self._id_sequence[-1]]
        p = Point(g[-1])
        p.set_srid(g.get_srid())
        return p

    def __repr__(self):
        return str(self.od_taz)