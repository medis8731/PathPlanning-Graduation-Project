import unittest
from Graph import Graph

class testEnviroment(unittest.TestCase):

    def test_sum(self):
        start =(3,3)
        end = (50,50)
        mapDim = (100,100)
        obsDim = (6,6)
        obsNum = 50
        env = Graph(start,end,mapDim,obsDim,obsNum)
        env.createGraph()
        env.createRandomObstacles()   
        env.createRandomNodes()
        env.connectAllNodes()
        env.showMap(True)  

    def test_random_node_1(self):
        start =(3,3)
        end = (50,50)
        mapDim = (100,100)
        obsDim = (6,6)
        obsNum = 50
        env = Graph(start,end,mapDim,obsDim,obsNum)
        env.createGraph()
        env.createRandomObstacles()  
        env.RRT_star(1000,5)     
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