import numpy as np
import scipy
import pdb
from itertools import groupby

from networks.GridNetwork import GridNetwork
from sensors.SensorConfiguration import SensorConfiguration

__author__ = 'yuanchenyang'

DATA_PREFIX = '/home/chenyang/src/megacell/data/'

def generate_synthetic(fname, grid_config, sensor_config, save=True):

    TN = GridNetwork(**grid_config)
    S = SensorConfiguration(**sensor_config)
    S.sample_sensors(TN)
    data = S.export_matrices(TN)

    A, U, x, b, f = data['A'], data['U'], data['x_true'], data['b'], data['f']
    assert np.linalg.norm(A.dot(x) - b) < 1e-10, 'Ax != b'
    assert np.linalg.norm(U.dot(x) - f) < 1e-10, 'Ux != f'
    row_c, col_c, vals = scipy.sparse.find(U)

    # permute columns to be in block diagonal format
    A = A[:, col_c]
    U = U[:, col_c]
    x = x[col_c]
    block_sizes = [len(list(g)) for _, g in groupby(row_c)]
    block_starts = []
    s = 0
    for i in block_sizes:
        block_starts.append(s)
        s += i

    assert np.linalg.norm(A.dot(x) - b) < 1e-10, 'Ax != b'
    assert np.linalg.norm(U.dot(x) - f) < 1e-10, 'Ux != f'
    assert sum(block_sizes) == len(x), 'Block sizes incompatible with x!'

    new_mat = { 'A': A
              , 'U': U
              , 'x_true': x
              , 'b': b
              , 'f': f
              , 'blocks': np.array(block_sizes)
              , 'block_starts': np.array(block_starts)
              }

    if save:
        scipy.io.savemat(fname, new_mat, oned_as='column')
    return new_mat

def main():
    sensor_config = { 'num_link' : np.inf
                    , 'num_OD' : np.inf
                    , 'num_cellpath_NB' : 10 # Number of cell towers
                    , 'num_cellpath_NL' : 0
                    , 'num_cellpath_NS' : 0
                    , 'num_linkpath' : 2 # license plate detectors; make small
                    }

    grid_config = { 'ncol': 5
                  , 'nrow': 5
                  , 'nodroutes': 2
                  , 'nnz_oroutes': 2
                  , 'o_flow' : 0.8
                  , 'concentration' : None
                  , 'myseed': None
                  }

    fname =  DATA_PREFIX + 'test_mat.mat'
    generate_synthetic(fname, grid_config, sensor_config, save=True)

main()
