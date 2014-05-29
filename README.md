synthetic-traffic
=================

We are working on producing a set of synthetic urban traffic networks and corresponding data for benchmarking and evaluation purposes.

For example usage, please see:

[Convex optimization for traffic assignment](https://github.com/cathywu/traffic-estimation)

[Bayesian inference for traffic assignment](https://github.com/cathywu/traffic-estimation-bayesian)

[Compressive sensing for traffic assignment](https://github.com/pcmoritz/traffic-project)

Also, see our [contributors](AUTHORS.md)!

General dependencies
-------------------
    
We use Python 2.7.

    scipy
    ipython
    matplotlib
    delegate
    
Contents
--------
[Toy networks](#toynetworks)

[Grid networks](#gridnetworks)

[Waypoints](#waypoints)

<a name="toynetworks"></a>
Toy networks
------------

Coming soon!

<a name="gridnetworks"></a>
Grid networks
-------------

Dependencies for grid networks

    networkx

Usage

    python static_matrix.py --prefix '' --num_rows <# ROWS OF STREETS> \
        --num_cols <# COLUMNS OF STREETS> \
        --num_routes_per_od <# ROUTES BETWEEN ODS> \
        --num_nonzero_routes_per_o <# ROUTES WITH NONZERO FLOW PER OD>

Example

    python static_matrix.py --prefix '' --num_rows 2 --num_cols 2 \
        --num_routes_per_od 3 --num_nonzero_routes_per_o 3

<a name="waypoints"></a>
Waypoints
---------

Dependencies for waypoint

    pyshp

Load map via Shapefile

    run -i find.py

Find new roads of interest

    roads = find('210',sf,shapes,verbose=True)

Generate waypoints

    run -i Waypoint.py

