import unittest
from Graph import Graph
from matplotlib import pyplot as plt
class testEnviroment(unittest.TestCase):

    def test_singleGoal(self):
        start =(3,3)
        end = [(50,50)]
        mapDim = (100,100)
        obsDim = (6,6)
        obsNum = 50
        env = Graph(start,end,mapDim,obsDim,obsNum)
        env.createGraph()
        env.createRandomObstacles()   
        # env.createRandomNodes()
        # env.connectAllNodes()
        env.showMap(True) 

    def test_TwoGoals(self):
        start =(3,3)
        end = [(50,50),(80,80)]
        mapDim = (100,100)
        obsDim = (6,6)
        obsNum = 50
        env = Graph(start,end,mapDim,obsDim,obsNum)
        env.createGraph()
        env.createRandomObstacles()   
        env.showMap(True)      

    def test_RRT_singleGoal(self):
        start =(3,3)
        end = (50,50)
        mapDim = (100,100)
        obsDim = (6,6)
        obsNum = 50
        env = Graph(start,[end],mapDim,obsDim,obsNum)
        env.createGraph()
        env.createRandomObstacles()  
        env.RRT_star(1000,5)     
        env.showMap() 

    def test_RRT_twoGoals(self):
        start =(3,3)
        end = (50,50)
        mapDim = (100,100)
        obsDim = (6,6)
        obsNum = 50
        env = Graph(start,[end,(80,80)],mapDim,obsDim,obsNum)
        env.createGraph()
        env.createRandomObstacles()  
        env.RRT_star(1000,5)     
        env.showMap() 

    def test_RRT_MultipleGoals(self):
        start =(3,3)
        end = (50,50)
        mapDim = (100,100)
        obsDim = (6,6)
        obsNum = 50
        env = Graph(start,[end,(80,80),(10,70),(15,20),(90,10)],mapDim,obsDim,obsNum)
        env.createGraph()
        env.createRandomObstacles()  
        env.RRT(500,5)     
        env.showMap(True,env.path) 
    
    def testFullGraph(self):
        listt = [(5,5),(50,50),(80,80),(10,70),(15,20),(90,10)]
        mapDim = (100,100)
        obsDim = (6,6)
        obsNum = 50
        
        for i in range(len(listt)):
            start = listt[i]
            goal = listt.copy()
            goal.pop(i)
            env = Graph(start,goal,mapDim,obsDim,obsNum)
            env.createGraph()
            env.createRandomObstacles()  
            env.RRT(1000,5)     
            env.showMap(True,env.path,i) 
        plt.show() 
if __name__ == '__main__':
    unittest.main()
