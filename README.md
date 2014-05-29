synthetic-traffic
=================

We aim to create a set of synthetic networks and traffic data for benchmarking and evaluation purposes.

General dependencies
    
    scipy
    ipython
    matplotlib
    delegate
    
Contents

[Toy networks](#toynetworks)
[Grid networks](#gridnetworks)
[Waypoints](#waypoints)

<a name="toynetworks"></a>

    Coming soon!

<a name="gridnetworks"></a>

Dependencies for grid networks

    networkx

<a name="waypoints"></a>

Dependencies for waypoint

    pyshp

Load map via Shapefile

    run -i find.py

Find new roads of interest

    roads = find('210',sf,shapes,verbose=True)

Generate waypoints

    run -i Waypoint.py
