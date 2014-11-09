from route import Route

class RouteCreator:
    def __init__(self, similarity_ratio):
        self.similarity_ratio = similarity_ratio
        self.trajectories = set()

    def extract_route(self):
        max_matching_trajectories = []
        prototype_trajectory = None

        for t1 in self.trajectories:
            matches = [t2 for t2 in self.trajectories if (t1.match_percent(t2) > self.similarity_ratio)]

            if len(matches) > len(max_matching_trajectories):
                max_matching_trajectories = matches
                prototype_trajectory = t1

        return prototype_trajectory, max_matching_trajectories

    def set_trajectories(self, trajectories):
        self.trajectories = set(trajectories)

    @staticmethod
    def make_route(t, matches):
        return Route(t.od_taz,t, len(matches))

    def extract_all_routes(self):
        l = list()

        # this is an extra stopping condition in case there are strange trajectories
        # that have implementations which are not self matching
        trajectory_size = 0

        while len(self.trajectories) != 0 and len(self.trajectories) != trajectory_size:
            trajectory_size = len(self.trajectories)
            t, matches = self.extract_route()
            if (t != None):
                l.append(self.make_route(t, matches))
            self.trajectories.difference_update(matches)

        return l
