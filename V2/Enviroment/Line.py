import numpy as np
class Line():
    def __init__(self, p0, p1):
        self.p = np.array(p0)
        
    def Intersection(self,node1,node2,recXY,obsDim) :
        x1,y1 = node1
        x2,y2 = node2
        cX,cY = recXY
        minX = cX
        minY = cY
        w,h = obsDim
        maxX = cX + w
        maxY = cY + h
        # Completely outside.
        if ((x1 <= minX and x2 <= minX) or (y1 <= minY and y2 <= minY) or (x1 >= maxX and x2 >= maxX) or (y1 >= maxY and y2 >= maxY)):
            return False
        if(x2-x1==0):
            return True
        if(y2-y1==0):
            return True     
        m = (y2 - y1) / (x2 - x1)

        y = m * (minX - x1) + y1
        if (y > minY and y < maxY): return True

        y = m * (maxX - x1) + y1
        if (y > minY and y < maxY): return True

        x = (minY - y1) / m + x1
        if (x > minX and x < maxX): return True

        x = (maxY - y1) / m + x1
        if (x > minX and x < maxX): return True

        return False; 


