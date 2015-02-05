import ipdb
import warnings

import numpy as np
from scipy.sparse import csr_matrix, coo_matrix

__author__ = 'cathywu, jeromethai'

# Helper functions
# -------------------------------------
def to_np(X):
    return np.array(X).squeeze()

def to_sp(X):
    return csr_matrix((to_np(X.V),(to_np(X.I),to_np(X.J))), shape=X.size)

# Clean matrix  wrapper
def matrix(x):
    return np.atleast_2d(np.squeeze(np.array(x)))

def deprecated(func):
    '''This is a decorator which can be used to mark functions
    as deprecated. It will result in a warning being emitted
    when the function is used.'''
    def new_func(*args, **kwargs):
        warnings.warn("Call to deprecated function {}.".format(func.__name__),
                      category=DeprecationWarning)
        return func(*args, **kwargs)
    new_func.__name__ = func.__name__
    new_func.__doc__ = func.__doc__
    new_func.__dict__.update(func.__dict__)
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

def distance_on_unit_sphere(lat1, long1, lat2, long2):
    import math

    # Convert latitude and longitude to
    # spherical coordinates in radians.
    degrees_to_radians = math.pi/180.0

    # phi = 90 - latitude
    phi1 = (90.0 - lat1)*degrees_to_radians
    phi2 = (90.0 - lat2)*degrees_to_radians

    # theta = longitude
    theta1 = long1*degrees_to_radians
    theta2 = long2*degrees_to_radians

    # Compute spherical distance from spherical coordinates.

    # For two locations in spherical coordinates
    # (1, theta, phi) and (1, theta, phi)
    # cosine( arc length ) =
    #    sin phi sin phi' cos(theta-theta') + cos phi cos phi'
    # distance = rho * arc length

    cos = (math.sin(phi1)*math.sin(phi2)*math.cos(theta1 - theta2) +
           math.cos(phi1)*math.cos(phi2))
    arc = math.acos( cos )

    # Remember to multiply arc by the radius of the earth
    # in your favorite set of units to get length.
    #return 3960.*arc to get in miles
    #return 6373.*arc to get in km
    return arc
