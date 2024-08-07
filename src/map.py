class Cuboid:
    def __init__(self, xmin, ymin, zmin, xmax, ymax, zmax, r, g, b):
        self.xmin = xmin
        self.xmax = xmax
        self.ymin = ymin
        self.ymax = ymax
        self.zmin = zmin
        self.zmax = zmax
        self.r = r
        self.g = g
        self.b = b

    def isPointInsideCuboid(self, point, buffer = 0.3):
        if (self.xmin - buffer < point[0] and self.xmax + buffer > point[0]) and \
           (self.ymin - buffer < point[1] and self.ymax + buffer > point[1]) and \
           (self.zmin - buffer < point[2] and self.zmax + buffer > point[2]):
            return True
        else:
            return False
        
    def getCuboidColor(self):
        return self.r,self.g,self.b

class Map:
    def __init__(self):
        self.boundary = None
        self.obstacles = []

    def set_buffer(self, buffer):
        self.buffer = buffer

    def setMapBoundary(self, boundaryCuboid):
        self.boundary = boundaryCuboid
        self.size_x = self.boundary.xmax - self.boundary.xmin
        self.size_y = self.boundary.ymax - self.boundary.ymin
        self.size_z = self.boundary.zmax - self.boundary.zmin
    
    def addObstacle(self, obstacleCuboid):
        self.obstacles.append(obstacleCuboid)

    def isPointInMapBoundary(self, point):
        return self.boundary.isPointInsideCuboid(point)
    
    def isPointInCollision(self, point):
        for obstacle in self.obstacles:
            if obstacle.isPointInsideCuboid(point, self.buffer):
                return True
        return False