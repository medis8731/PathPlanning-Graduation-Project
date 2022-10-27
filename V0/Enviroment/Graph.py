import random
from matplotlib import pyplot as plt
from matplotlib.patches import Circle, Rectangle
import numpy as np
from Enviroment import Enviroment

class Graph(Enviroment):
    def createGraph(self):

        self.vertices = [self.start]
        self.edges = []
        self.success = False

        self.vex2idx = {self.start:0}
        self.neighbors = {0:[]}
        self.distances = {0:0.}

        self.sx = self.goal[0] - self.start[0]
        self.sy = self.goal[1] - self.start[1]

        self.nodeR = 1

    def randomVertex(self):
        x = int(random.uniform(0, self.mapw))
        y = int(random.uniform(0, self.maph))
        return (x,y)

    def distance(self,x, y):
        return np.linalg.norm(np.array(x) - np.array(y))

    def isInObstacle(self,vex):
        obstacles = self.obs
        for obs in obstacles:
            rectang = Rectangle((obs[0], obs[1]), self.obsdim[1], self.obsdim[0])
            center = rectang.get_center()
            if self.circleInterRec(vex[0],vex[1],0.5,center[0],center[1],self.obsdim[0],self.obsdim[1]):
                return True
        return False

    def createRandomNodes(self):
        for i in range(100):
            node = self.randomVertex()
            if(self.isInObstacle(node)):
                continue
            self.vertices.append(node)

    def showMap(self,showWhileadding=False):
        self.fig ,self.ax = plt.subplots()
        self.ax.set_xlim(0,self.mapw)
        self.ax.set_ylim(0,self.maph)
        self.ax.add_patch(Circle(self.start,self.startR,color=self.startColor))
        self.ax.add_patch(Circle(self.goal,self.goalR,color= self.goalColor))
        for obs in self.obs:
        #add rectangle to plot
            self.ax.add_patch(Rectangle((obs[0], obs[1]), self.obsdim[1], self.obsdim[0]))
            # if(showWhileadding):plt.pause(0.05)
        for nodes in self.vertices[1:]:
            self.ax.add_patch(Circle((nodes[0],nodes[1]),0.5,color="#FF0000"))
            plt.pause(0.05)
        plt.show()                 
start =(3,3)
end = (50,50)
mapDim = (100,100)
obsDim = (6,6)
obsNum = 50
env = Graph(start,end,mapDim,obsDim,obsNum)
env.createGraph()
env.createRandomObstacles()   
env.createRandomNodes()
env.showMap(True)     