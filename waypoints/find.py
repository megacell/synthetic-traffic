import shapefile
# See documentation: https://code.google.com/p/pyshp/wiki/PyShpDocs

# find records with a certain road name
def find(names, sf, shapes,verbose=False,exact=False):
    records = sf.iterRecords()
    if sf.fields:
        index = [i for (i,x) in enumerate(sf.fields) if x[0] == 'FULLNAME'][0] - 1
    else:
        index = 17 # 17: full road name
    r = []
    for i,record in enumerate(records):
        if exact:
            r.extend([(i,record[index]) for x in names if x == record[index]])
        else:
            r.extend([(i,record[index]) for x in names if x in record[index]])
    if verbose:
        print [x for (i,x) in r]

    return [(x,shapes[i]) for (i,x) in r]

def save(c,roads):
    import pickle
    pickle.dump(roads,open('%s/%s' % (c.DATA_DIR,c.ROAD_FILE),'w'))

if __name__ == "__main__":
    import config as c
    sf = shapefile.Reader("%s/tigerroadsWGS84/tigerroadsWGS84" % c.DATA_DIR)
    shapes = sf.shapes()
    import ipdb
    ipdb.set_trace()
    # Example of how to find new roads
    # roads = find(['605'],sf,shapes,verbose=True)

    roads = []
    roads = find(c.road_names,sf,shapes)
    # for x in c.road_names:
    #     roads.extend(find(x,sf,shapes))
    save(c,roads)
