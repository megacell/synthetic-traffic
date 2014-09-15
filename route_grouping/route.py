class Route:
    def __init__(self, od_taz_id, trajectory, agent_count):
        self._trajectory = trajectory
        self._agent_count = agent_count
        self.od_taz_id = od_taz_id

    def __str__(self):
        return str(self.od_taz_id) + " Number of agents:" + str(self._agent_count)

    def __repr__(self):
        return self.__str__()

    def convert_to_dictionary(self):
        d = dict()
        d['(orig_taz, dest_taz)'] = str(self.od_taz_id)
        d['number_of_agents'] = str(self._agent_count)
        d['trajectory'] = self._trajectory._id_sequence
        return d
