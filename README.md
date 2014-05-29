synthetic-traffic
=================

We are working on producing a set of synthetic urban traffic networks and corresponding data for benchmarking and evaluation purposes.

For example usage, please see:

[Convex optimization for traffic assignment](https://github.com/cathywu/traffic-estimation)\n
[Bayesian inference for traffic assignment](https://github.com/cathywu/traffic-estimation-bayesian)\n
[Compressive sensing for traffic assignment](https://github.com/pcmoritz/traffic-project)

General dependencies
-------------------
    
    scipy
    ipython
    matplotlib
    delegate
    
Contents
--------
[Toy networks](#toynetworks)\n
[Grid networks](#gridnetworks)\n
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
