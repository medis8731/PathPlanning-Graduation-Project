from matplotlib import pyplot as plt
import matplotlib
from matplotlib.patches import Circle, Rectangle
from matplotlib import collections  as mc
import random

class Solution(): 
    def __init__(self, paths, costs, items,obstacles):
        self.paths = paths
        self.costs = costs
        self.items = items
        self.obs = obstacles
        number_of_colors = len(items)
        self.colors = ["#"+''.join([random.choice('0123456789ABCDEF') for j in range(6)]) for i in range(number_of_colors)]
    
    def showAllPaths(self):
        # plt.figure(i)
        self.fig ,self.ax = plt.subplots()
        self.ax.set_xlim(0,100)
        self.ax.set_ylim(0,100)
        # self.ax.add_patch(Circle(self.start,self.startR,color=self.startColor))
        patches = [plt.Circle(center, 1) for center in self.items]
        coll = matplotlib.collections.PatchCollection(patches, facecolors= '#241571')
        self.ax.add_collection(coll)
        i =0
        for obs in self.obs:
        #add rectangle to plot
            self.ax.add_patch(Rectangle((obs[0], obs[1]), 6, 6))
        for start in self.paths.keys():
            color = self.colors[i]
            for goal in  self.paths[start]:
                pathC = self.paths[start][goal]
                if(pathC is None): continue
                paths = [(pathC[i], pathC[i+1]) for i in range(len(pathC)-1)]
                lc2 = mc.LineCollection(paths, colors=color, linewidths=2)
                # print("trying")
                self.ax.add_collection(lc2)
            i+=1     
        plt.show()    