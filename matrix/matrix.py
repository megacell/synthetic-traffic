import numpy as np
import scipy
from networks.GridNetwork import GridNetwork
from sensors.SensorConfiguration import SensorConfiguration

__author__ = 'yuanchenyang'

DATA_PREFIX = '/home/chenyang/src/megacell/data/'

def generate_synthetic(fname, config):
    o_flow = 1
    concentration = 0.1

    TN = GridNetwork(o_flow=o_flow, concentration=concentration)
    S = SensorConfiguration(**config)
    S.sample_sensors(TN)
    data = S.export_matrices(TN)
    import pdb; pdb.set_trace()
    scipy.io.savemat(fname, data, oned_as='column')
    return data

def main():
    config = {
        'num_link' : np.inf,
        'num_OD' : np.inf,
        'num_cellpath_NB' : 20, # Number of cell towers
        'num_cellpath_NL' : 0,
        'num_cellpath_NS' : 0,
        'num_linkpath' : 50 # license plate detectors; make small
    }

    fname =  DATA_PREFIX + 'test_mat.mat'
    generate_synthetic(fname, config)

main()
