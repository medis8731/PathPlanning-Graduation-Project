import random
from turtle import circle, color
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

        #Set radious of nodes and start and end
        self.goalR = 2
        self.startR = 2
        self.nodeR = 2

    def makeRandomRect(self):
        uppercornerx = int(random.uniform(1, self.mapw - self.obsdim[0]))
        uppercornery = int(random.uniform(1, self.maph -  self.obsdim[1]))
        return (uppercornerx, uppercornery)

    def createRandomObstacles(self):
        for i in range(0,self.obsnum):
            self.obs.append(self.makeRandomRect())
    
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

