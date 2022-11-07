import unittest
from Graph import Graph

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
        # env.createRandomNodes()
        # env.connectAllNodes()
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
        env.RRT(1000,5)     
        env.showMap(True,env.path) 

    def test_RRT_twoGoals(self):
        start =(3,3)
        end = (50,50)
        mapDim = (100,100)
        obsDim = (6,6)
        obsNum = 50
        env = Graph(start,[end,(80,80)],mapDim,obsDim,obsNum)
        env.createGraph()
        env.createRandomObstacles()  
        env.RRT(1000,5)     
        env.showMap(True,env.path) 
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
if __name__ == '__main__':
    unittest.main()

# start =(3,3)
# end = (50,50)
# mapDim = (100,100)
# obsDim = (6,6)
# obsNum = 50
# env = Graph(start,end,mapDim,obsDim,obsNum)
# env.createGraph()
# env.createRandomObstacles()   
# env.createRandomNodes()
# env.connectAllNodes()
# env.showMap(True)       