import random
from matplotlib import pyplot as plt
from matplotlib.patches import Circle, Rectangle
import numpy as np
from Enviroment import Enviroment
from matplotlib import collections  as mc
from Line import Line

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

    def add_vex(self, pos):
        try: #Check to see if Vertex already exists 
            idx = self.vex2idx[pos]
        except: #if it doesnt exist 
            idx = len(self.vertices) #The id for this vertex would be the number of vertices +1
            self.vertices.append(pos) 
            self.vex2idx[pos] = idx
            self.neighbors[idx] = []
        return idx

    def randomVertex(self):
        x = int(random.uniform(0, self.mapw))
        y = int(random.uniform(0, self.maph))
        return (x,y)

    # def randomVertex(self):
    #     rx = random.random()
    #     ry = random.random()
    #     posx = self.start[0] - (self.sx / 2.) + rx * self.sx * 2
    #     posy = self.start[1] - (self.sy / 2.) + ry * self.sy * 2
    #     return posx, posy

    def add_edge(self, idx1, idx2, cost):
        self.edges.append((idx1, idx2))
        self.neighbors[idx1].append((idx2, cost))
        self.neighbors[idx2].append((idx1, cost))   

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
        for i in range(200):
            node = self.randomVertex()
            while(self.isInObstacle(node)):
                node = self.randomVertex()

            self.add_vex(node)
    def connectAllNodes(self):
        vertices = self.vertices.copy()
        for node1 in self.vertices[0:50]:
            for node2 in vertices[51:]:
                if(not self.isThroughObstacle(node1,node2)):
                    self.add_edge(self.vex2idx[node1],self.vex2idx[node2],0)
                    
    def distance(self,x, y):
        return np.linalg.norm(np.array(x) - np.array(y))

    def newVertex(self,randvex, nearvex, stepSize):
        dirn = np.array(randvex) - np.array(nearvex)
        length = np.linalg.norm(dirn)
        dirn = (dirn / length) * min (stepSize, length)

        newvex = (nearvex[0]+dirn[0], nearvex[1]+dirn[1])
        return newvex   

    def nearest(self, vex):
        Nvex = None
        Nidx = None
        minDist = float("inf")

        for idx, v in enumerate(self.vertices):
            if self.isThroughObstacle(v,vex):
                continue

            dist = self.distance(v, vex)
            if dist < minDist:
                minDist = dist
                Nidx = idx
                Nvex = v

        return Nvex, Nidx

    def isThroughObstacle(self,node1,node2):
        line = Line(node1, node2)
        for obs in self.obs:
            #node1,node2,recXY,w,h
            if line.Intersection(node1,node2,obs,self.obsdim):
                return True
        return False

    # def RRT(self, n_iter, stepSize):
    #     for _ in range(n_iter):
    #         randvex = self.randomVertex()
    #         if self.isInObstacle(randvex):
    #             continue

    #         nearvex, nearidx = self.nearest(randvex)
    #         if nearvex is None:
    #             continue

    #         newvex = self.newVertex(randvex, nearvex, stepSize)

    #         newidx = self.add_vex(newvex)
    #         dist = self.distance(newvex, nearvex)
    #         self.add_edge(newidx, nearidx, dist)

    #         dist = self.distance(newvex, self.goal)
    #         if dist < 2 * self.goalR:
    #             endidx = self.add_vex(self.goal)
    #             self.add_edge(newidx, endidx, dist)
    #             self.success = True
    #             print('success')
    #             break
    def RRT(self, n_iter, stepSize):
        for iteration in range(n_iter):
            if iteration % 10 == 0:   
                randvex = self.goal
            else:
                randvex = self.randomVertex()    
            if self.isInObstacle(randvex):
                continue

            nearvex, nearidx = self.nearest(randvex)
            if nearvex is None:
                continue

            newvex = self.newVertex(randvex, nearvex, stepSize)

            newidx = self.add_vex(newvex)
            dist = self.distance(newvex, nearvex)
            self.add_edge(newidx, nearidx, dist)

            dist = self.distance(newvex, self.goal)
            if dist < 2 * self.goalR:
                endidx = self.add_vex(self.goal)
                self.add_edge(newidx, endidx, dist)
                self.success = True
                print('success')
                break    
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
            # plt.pause(0.05)
        lines = [(self.vertices[edge[0]], self.vertices[edge[1]]) for edge in self.edges]
        lc = mc.LineCollection(lines, colors='green', linewidths=2)
        self.ax.add_collection(lc)    
        plt.show()
     
# if __name__ == '__main__':

#     start =(3,3)
#     end = (50,50)
#     mapDim = (100,100)
#     obsDim = (6,6)
#     obsNum = 50
#     env = Graph(start,end,mapDim,obsDim,obsNum)
#     env.createGraph()
#     env.createRandomObstacles()  
#     env.RRT(500,5)     
#     # env.createRandomNodes()
#     # env.connectAllNodes()
#     env.showMap(True) 
 