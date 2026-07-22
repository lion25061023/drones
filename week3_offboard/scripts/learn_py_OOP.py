import math

class WayPoint:
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z
    def distance_to(self, other):
        #具体的数学公式我不知道这里怎么打
        #具体应该是self.x和other.x这样相减，然后想加平方起来吧
        dx = self.x - other.x
        dy = self.y - other.y
        dz = self.z - other.z
        re = math.sqrt(dx**2+dy**2+dz**2)
        return re

a = WayPoint(1,1,1)
b = WayPoint(0,0,0)

print (a.distance_to(b)) #记得self是自己，所以a.distance_to(b)就是a是self，b是other，a不用打进去的
