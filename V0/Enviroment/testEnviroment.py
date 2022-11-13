import unittest
from Graph import Graph
import time
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
        # env.connectAllNodes()
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

        start_time = time.time()
        env.RRT_star(500,5)  
        print("--- %s seconds ---" % (time.time() - start_time))    
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