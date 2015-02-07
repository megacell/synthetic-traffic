import ipdb
import warnings

import numpy as np
import scipy.sparse as sps
from scipy.sparse import csr_matrix, coo_matrix
import functools

__author__ = 'cathywu, jeromethai'

# Helper functions
# -------------------------------------
def to_np(X):
    return np.array(X).squeeze()

def to_sp(X):
    if X is None:
        return None
    return csr_matrix((to_np(X.V),(to_np(X.I),to_np(X.J))), shape=X.size)

# Clean matrix  wrapper
def matrix(x):
    return np.atleast_2d(np.squeeze(np.array(x)))

# Clean array wrapper
def array(x):
    return np.atleast_1d(np.squeeze(np.array(x)))

# Clean sparse matrix wrapper
def sparse(A):
    if type(A) == np.ndarray:
        return sps.csr_matrix(A)
    return A.tocsr()

def sps2array(x):
    return array(x.todense())

def deprecated(func):
    '''This is a decorator which can be used to mark functions
    as deprecated. It will result in a warning being emitted
    when the function is used.'''

    @functools.wraps(func)
    def new_func(*args, **kwargs):
        warnings.warn_explicit(
            "Call to deprecated function {}.".format(func.__name__),
            category=Warning,
            filename=func.func_code.co_filename,
            lineno=func.func_code.co_firstlineno + 1
        )
        return func(*args, **kwargs)
    return new_func

def simplex(nroutes, traj, flows):
    """
    Build simplex matrix from nroutes (n), trajectories (m), and trajectory
    flows

    We represent each trajectory as its own row of "1"s (X). We represent the
    respective trajectory flow vector (r).

    Applicable to cellpath and linkpath flows

    :param nroutes: number of routes
    :param traj: dictionary keyed on the trajectory, valued on the
                 respective list of routes
    :param flows: trajectory flows
    :return:
    """
    from cvxopt import matrix, spmatrix
    m, n = nroutes, len(traj)
    I, J, r = [], [], matrix(0.0, (n,1))
    for i, path_ids in enumerate(traj.itervalues()):
        r[i] = flows[i]
        for id in path_ids:
            I.append(i)
            J.append(id)
    X = coo_matrix(([1.0] * len(I),(I,J)), shape=(n,m)).tocsr()
    r = to_np(r)
    return X, r

