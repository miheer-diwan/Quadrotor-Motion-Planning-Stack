from map import Cuboid, Map

class MapReader:
    def __init__(self, filepath) -> None:
        self.map = Map()
        with open(filepath, 'r') as f:
            lines = f.readlines()

        for line in lines:
            tokens = line.split()
            if len(tokens) == 0:
                continue
            if tokens[0] == 'boundary':
                xmin, ymin, zmin, xmax, ymax, zmax = map(float, tokens[1:])
                boundaryCuboid = Cuboid(xmin, ymin, zmin, xmax, ymax, zmax, 0, 0, 0)
                self.map.setMapBoundary(boundaryCuboid)
                print("map boundary set")
            elif tokens[0] == 'block':
                xmin, ymin, zmin, xmax, ymax, zmax, r, g, b = map(float, tokens[1:])
                obstacleCuboid = Cuboid(xmin, ymin, zmin, xmax, ymax, zmax, r, g, b)
                self.map.addObstacle(obstacleCuboid)
    
    def getMap(self):
        return self.map
    
