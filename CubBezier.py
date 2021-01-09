import random
import matplotlib.pyplot as plt
import math
from scipy.optimize import minimize, rosen, rosen_der
import random

class Point(object):
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def random(self, min= 0, max= 1):
        self.x = random.uniform(min,max)
        self.y = random.uniform(min,max)

class CubicBezier(object):
    def __init__(self, p1x= 0, p1y= 0, p2x= 0, p2y= 0, p3x= 0, p3y= 0, p4x= 0, p4y= 0):
        self.p1 = Point(p1x, p1y)
        self.p2 = Point(p2x, p2y)
        self.p3 = Point(p3x, p3y)
        self.p4 = Point(p4x, p4y)
        self.obstacles = []

    def random(self,min= 0, max= 1):
        'Create a random Bezier curve within [min, max] limits. Default [0,1].'
        self.p1.random(min, max)
        self.p2.random(min, max)
        self.p3.random(min, max)
        self.p4.random(min, max)

    def max_k(self, granuality=100):
        'Calculate maximal curvature of the Bezier curve.'
        k = 0
        for t in range(0, granuality):
            t = t / granuality
            x_d = 3 * ((1 - t) ** 2) * (self.p2.x - self.p1.x) + 6 * (1 - t) * t * (self.p3.x - self.p2.x) + 3 * (t ** 2) * (
                        self.p4.x - self.p3.x)
            y_d = 3 * ((1 - t) ** 2) * (self.p2.y - self.p1.y) + 6 * (1 - t) * t * (self.p3.y - self.p2.y) + 3 * (t ** 2) * (
                        self.p4.y - self.p3.y)
            x_dd = 6 * (1 - t) * (self.p3.x - 2 * self.p2.x + self.p1.x) + 6 * t * (self.p4.x - 2 * self.p3.x + self.p2.x)
            y_dd = 6 * (1 - t) * (self.p3.y - 2 * self.p2.y + self.p1.y) + 6 * t * (self.p4.y - 2 * self.p3.y + self.p2.y)
            k = max(k,abs(x_d*y_dd - y_d*x_dd)/math.pow(x_d**2 + y_d**2, 3/2))
        return k

    def calc_curve(self, granuality=100):
        'Calculate the Bezier curve with the given granuality.'
        B_x = []
        B_y = []
        for t in range(0, granuality):
            t = t / granuality
            x = ((1 - t) ** 3) * self.p1.x + 3 * ((1 - t) ** 2) * t * self.p2.x + 3 * (1 - t) * (t ** 2) * self.p3.x\
                + (t ** 3) * self.p4.x
            y = ((1 - t) ** 3) * self.p1.y + 3 * ((1 - t) ** 2) * t * self.p2.y + 3 * (1 - t) * (t ** 2) * self.p3.y\
                + (t ** 3) * self.p4.y
            B_x.append(x)
            B_y.append(y)
        return [B_x, B_y]

    def plot(self, granuality=100):
        'Plot the Bezier curve.'
        B = self.calc_curve(granuality)
        plt.plot(B[0], B[1])
        plt.scatter([self.p1.x,self.p2.x,self.p3.x,self.p4.x], [self.p1.y,self.p2.y,self.p3.y,self.p4.y])
        for i in range(len(self.obstacles)):
            plt.gcf().gca().add_artist(plt.Circle((self.obstacles[i][0].x, self.obstacles[i][0].y), self.obstacles[i][1], color='r'))
        plt.axis('equal')
        plt.show()


    def arc_len(self, granuality=1000):
        'Calculate the arc-length of the Bezier curve.'
        B = self.calc_curve(granuality=granuality)
        a_l = 0

        for i in range(1,len(B[0])):
            a_l += math.sqrt((B[0][i]-B[0][i-1])**2 + (B[1][i]-B[1][i-1])**2)

        return a_l

    def optimize_k(self, granuality= 100, obs= True):
        'Optimize the Bezier curve to minimize the curvature. By setting obs=False, ignore the obstacles.'
        x0 = [0.0, 0.0, 0.0, 0.0]
        res = minimize(self.optimizer_k, x0, args= (granuality, obs, 1), method='Nelder-Mead', tol=1e-7)
        self.p2.x = self.p2.x + res.x[0]
        self.p2.y = self.p2.y + res.x[1]
        self.p3.x = self.p3.x + res.x[2]
        self.p3.y = self.p3.y + res.x[3]

    def optimizer_k(self,x, *args):
        'Curvature optimizer function.'
        granuality = args[0]
        obs = args[1]
        k_multiplier = args[2]
        o = CubicBezier()
        o.p1 = self.p1
        o.p2.x = self.p2.x+x[0]
        o.p2.y = self.p2.y+x[1]
        o.p3.x = self.p3.x + x[2]
        o.p3.y = self.p3.y + x[3]
        o.p4 = self.p4
        penalty = 0

        if obs:
            B = o.calc_curve(granuality)
            for i in range(len(B[0])):
                for j in range(len(self.obstacles)):
                    d = math.sqrt((B[0][i] - self.obstacles[j][0].x)**2 + (B[1][i] - self.obstacles[j][0].y)**2)
                    if d<self.obstacles[j][1]:
                        penalty += (self.obstacles[j][1]-d)*100
        return (o.max_k(granuality) + penalty)*k_multiplier

    def optimize_l(self, granuality= 100, obs= True):
        'Optimize the Bezier curve to minimize the arc-length. By setting obs=False, ignore the obstacles.'
        x0 = [0.0, 0.0, 0.0, 0.0]
        res = minimize(self.optimizer_l, x0, args=(granuality, obs, 1), method='Nelder-Mead', tol=1e-7)
        self.p2.x = self.p2.x + res.x[0]
        self.p2.y = self.p2.y + res.x[1]
        self.p3.x = self.p3.x + res.x[2]
        self.p3.y = self.p3.y + res.x[3]


    def optimizer_l(self,x, *args):
        'Arc-length optimizer function.'
        granuality = args[0]
        obs = args[1]
        l_multiplier = args[2]
        o = CubicBezier()
        o.p1 = self.p1
        o.p2.x = self.p2.x+x[0]
        o.p2.y = self.p2.y+x[1]
        o.p3.x = self.p3.x + x[2]
        o.p3.y = self.p3.y + x[3]
        o.p4 = self.p4

        penalty = 0
        if obs:
            B = o.calc_curve(granuality)
            for i in range(len(B[0])):
                for j in range(len(self.obstacles)):
                    d = math.sqrt((B[0][i] - self.obstacles[j][0].x)**2 + (B[1][i] - self.obstacles[j][0].y)**2)
                    if d<self.obstacles[j][1]:
                        penalty += (self.obstacles[j][1]-d)*100

        return (o.arc_len(granuality) + penalty)*l_multiplier

    def optimize(self, granuality=100, obs=True, l_multiplier=0.5, k_multiplier=0.5):
        """
        Optimize the Bezier curve to simultaniously minimize the arc-lenght and the curvature.
        Setting obs=False ignores the obstacles. l_multiplier and k_multiplier multiplies
        the outputs of their respective optimizer functions.
        """
        x0 = [0.0, 0.0, 0.0, 0.0]
        res = minimize(self.optimizer, x0, args=(granuality, obs, l_multiplier, k_multiplier), method='Nelder-Mead', tol=1e-7)
        self.p2.x = self.p2.x + res.x[0]
        self.p2.y = self.p2.y + res.x[1]
        self.p3.x = self.p3.x + res.x[2]
        self.p3.y = self.p3.y + res.x[3]

    def optimizer(self,x,*args):
        'Optimizer function of the arc-length and curvature simultanious optimization.'
        granuality = args[0]
        obs = args[1]
        l_multiplier = args[2]
        k_multiplier = args[3]

        return self.optimizer_l(x, granuality, obs, l_multiplier) + self.optimizer_k(x, granuality, obs, k_multiplier)

    def add_obstacle(self, x=0, y=0, radius=0):
        'Add an obstacle to the Bezier curve.'
        self.obstacles.append([Point(x,y), radius])

    def add_random_obstacle(self, min_x= 1, max_x= 0, min_y=1, max_y=0, min_radius=0.3, max_radius = 0.0):
        """Add a random obstacle to the Bezier curve. The obstacle will not cover the p1 and p4 points
        of the Bezier curve.
        """
        radius = random.uniform(min_radius,max_radius)

        d = 0
        x = 0
        y = 0
        while d<radius:
            x = random.uniform(min_x,max_x)
            y = random.uniform(min_y,max_y)
            d1 = math.sqrt((x - self.p1.x)**2 + (y - self.p1.y)**2)
            d2 = math.sqrt((x - self.p4.x) ** 2 + (y - self.p4.y) ** 2)
            d = min(d1,d2)

        self.obstacles.append([Point(x, y), radius])

    def clear(self):
        'Re-initialize the curve.'
        self.__init__()

