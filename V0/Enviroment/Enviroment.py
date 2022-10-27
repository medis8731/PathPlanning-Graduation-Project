import random
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon, Rectangle

class Enviroment:
    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        #Set locations and dimensions
        self.start = start
        self.goal = goal 
        self.MapDimensions = MapDimensions
        self.maph, self.mapw = self.MapDimensions
        self.obsdim = obsdim
        self.obsnum = obsnum
        self.obs = []

        #set color of start.goal and obstacles
        self.startColor = '#FFB6C1'
        self.goalColor = '#008000'

        #Set radius of nodes and start and end
        self.goalR = 2
        self.startR = 2
        self.nodeR = 2

    def makeRandomRect(self):
        uppercornerx = int(random.uniform(1, self.mapw - self.obsdim[0]))
        uppercornery = int(random.uniform(1, self.maph -  self.obsdim[1]))
        return (uppercornerx, uppercornery)

    def createRandomObstacles(self):
        for i in range(0, self.obsnum):
            rectang = None
            startgoalcol = True
            while startgoalcol:
                rectangCor = self.makeRandomRect()
                rectang = Rectangle((rectangCor[0], rectangCor[1]), self.obsdim[1], self.obsdim[0])
                center = rectang.get_center()
                if self.circleInterRec(self.start[0],self.start[1],self.startR,center[0],center[1],self.obsdim[1], self.obsdim[0]) \
                     or self.circleInterRec(self.goal[0],self.goal[1],self.goalR,center[0],center[1],self.obsdim[1], self.obsdim[0]):
                    startgoalcol = True
                else:
                    startgoalcol = False
            self.obs.append(rectangCor)
        return self.obs

    def recConrner(self,rectangle):
        coords = rectangle.get_bbox().get_points()
        return (coords[0][0],coords[1][1],coords[1][0],coords[0][1])

    def circleInterRec(self,cirCenterX,cirCenterY,cirR,rectX,rectY,recH,recW):
        circleDistanceX = abs(cirCenterX - rectX)
        circleDistanceY = abs(cirCenterY - rectY)

        if (circleDistanceX > (recW/2 + cirR)):  return False
        if (circleDistanceY > (recH/2 + cirR)) :  return False

        if (circleDistanceX <= (recW/2)): return True
        if (circleDistanceY <= (recH/2)) : return True

        cornerDistance_sq = (circleDistanceX - recW/2)**2 +(circleDistanceY - recH/2)**2

        return (cornerDistance_sq <= (cirR**2))

 

# env = Enviroment((3,3),(50,50),(100,100),(6,6),50)
# env.createRandomObstacles()
# env.showMap(True) 

