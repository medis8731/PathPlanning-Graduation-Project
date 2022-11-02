import numpy as np
class Line():
    def __init__(self, p0, p1):
        self.p = np.array(p0)
        self.dirn = np.array(p1) - np.array(p0)
        self.dist = np.linalg.norm(self.dirn)
        self.dirn = self.dirn/self.dist # normalize

    def path(self, t):
        return self.p + t * self.dirn
        
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
#   def Intersection(self, center, radius):
#     ''' Check line-sphere (circle) intersection '''
#     a = np.dot(self.dirn, self.dirn)
#     b = 2 * np.dot(self.dirn, self.p - center)
#     c = np.dot(self.p - center, self.p - center) - radius * radius

#     discriminant = b * b - 4 * a * c
#     if discriminant < 0:
#         return False

#     t1 = (-b + np.sqrt(discriminant)) / (2 * a)
#     t2 = (-b - np.sqrt(discriminant)) / (2 * a)

#     if (t1 < 0 and t2 < 0) or (t1 > self.dist and t2 > self.dist):
#         return False

#     return True 

