import numpy as np
import scipy
from pdb import set_trace as ST
from itertools import groupby

from networks.GridNetwork import GridNetwork
from sensors.SensorConfiguration import SensorConfiguration

__author__ = 'yuanchenyang'

DATA_PREFIX = '/home/chenyang/src/megacell/BSC-NNLS/python/experiments/'

def sanity_check(A, U, b, f, x, tolerance=1e-10):
    assert np.linalg.norm(A.dot(x) - b) < tolerance, 'Ax != b'
    assert np.linalg.norm(U.dot(x) - f) < tolerance, 'Ux != f'

def generate_synthetic(fname, grid_config, sensor_config, save=True):

    TN = GridNetwork(**grid_config)
    S = SensorConfiguration(**sensor_config)
    S.sample_sensors(TN)
    data = S.export_matrices(TN)

    # Import data
    A, U, x, b, f = data['A'], data['U'], data['x_true'], data['b'], data['f']
    U = U.todense()
    sanity_check(A, U, b, f, x)

    # Remove zero values of f
    remove_rows = [i for i, val in enumerate(f) if val == 0]
    U = np.delete(U, remove_rows, axis=0)
    f = np.delete(f, remove_rows, axis=0)
    sanity_check(A, U, b, f, x)

    # Permute columns to be in block diagonal format
    row_c, col_c, vals = scipy.sparse.find(U)
    A = A[:, col_c]
    U = U[:, col_c]
    x = x[col_c]
    sanity_check(A, U, b, f, x)

    # Remove size one blocks
    block_sizes = [len(list(g)) for _, g in groupby(row_c)]
    row, col = [], []
    i, j = 0, 0
    for i, block_size in enumerate(block_sizes):
        if block_size == 1:
            row.append(i)
            col.append(j)
        j += block_size
    for j in col:
        b -= x[j] * np.squeeze(A[:,j])
    U = np.delete(U,row,0)
    U = np.delete(U,col,1)
    A = np.delete(A,col,1)
    f = np.delete(f,row,0)
    x = np.delete(x,col,0)
    sanity_check(A, U, b, f, x)

    # Generate block_starts
    block_sizes = [bs for bs in block_sizes if bs != 1]
    block_starts = []
    s = 0
    for i in block_sizes:
        block_starts.append(s)
        s += i

    sanity_check(A, U, b, f, x)
    assert sum(block_sizes) == len(x), 'Block sizes incompatible with x!'

    print 'A: {} U: {}'.format(A.shape, U.shape)

    new_mat = { 'A': A
              , 'U': U
              , 'x_true': x
              , 'b': b
              , 'f': f
              , 'block_sizes': np.array(block_sizes)
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

    grid_config = { 'ncol': 7
                  , 'nrow': 7
                  , 'nodroutes': 2
                  , 'nnz_oroutes': 2
                  , 'o_flow' : 0.8
                  , 'concentration' : None
                  , 'myseed': None
                  }

    fname =  DATA_PREFIX + 'test_mat.mat'
    generate_synthetic(fname, grid_config, sensor_config, save=True)

main()
