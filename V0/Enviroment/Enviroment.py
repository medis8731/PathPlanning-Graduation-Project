import random
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon, Rectangle
import numpy as np
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

        #Set radious of nodes and start and end
        self.goalR = 2
        self.startR = 2
        self.nodeR = 2

    def makeRandomRect(self):
        uppercornerx = int(random.uniform(1, self.mapw - self.obsdim[0]))
        uppercornery = int(random.uniform(1, self.maph -  self.obsdim[1]))
        return (uppercornerx, uppercornery)

    def createRandomObstaclesV0(self):
        for i in range(0,self.obsnum):
            self.obs.append(self.makeRandomRect())

    def createRandomObstacles(self):
        obs = []
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
            obs.append(rectangCor)
        self.obs = obs.copy()
        return obs

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

        cornerDistance_sq = (circleDistanceX - recW/2)^2 +(circleDistanceY - recH/2)**2

        return (cornerDistance_sq <= (cirR**2))

    def showMap(self):
        self.fig ,self.ax = plt.subplots()
        self.ax.set_xlim(0,env.mapw)
        self.ax.set_ylim(0,env.maph)
        self.ax.add_patch(Circle(self.start,self.startR,color=self.startColor))
        self.ax.add_patch(Circle(self.goal,self.goalR,color= self.goalColor))
        for obs in env.obs:
        #add rectangle to plot
            self.ax.add_patch(Rectangle((obs[0], obs[1]), env.obsdim[1], env.obsdim[0]))
        plt.show()    

env = Enviroment((3,3),(50,50),(100,100),(6,6),50)
env.createRandomObstacles()
env.showMap()

