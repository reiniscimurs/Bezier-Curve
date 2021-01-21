from Bezier import CubicBezier, QuadBezier, Point
import matplotlib.pyplot as plt

path = []

def optimizer_k(obj,k):
    pass

def optimize_k(obj, k):
    'Optimize the cubic Bezier curve to minimize the curvature. By setting obs=False, ignore the obstacles.'
    x0 = [0.0, 0.0]
    res = minimize(optimizer_k, x0, args=(obj, k), method='Nelder-Mead', tol=1e-7)

for i in range(10):
    p = Point()
    p.random(0,i)
    path.append(p)

xs = [x.x for x in path]
ys = [y.y for y in path]
plt.axis('equal')
plt.plot(xs, ys)
plt.show()

bezier = []
bx = []
by = []
plt.plot(xs, ys)
for ct in range (10,1,-1):
    for i in range(len(path)-1):
        b = CubicBezier()
        b.p0 = path[i]

        if not i:
            print(i)
            x = path[i].x + ((path[i + 1].x - path[i].x) / 2)
            y = path[i].y + ((path[i + 1].y - path[i].y) / 2)
            b.p1 = Point(x, y)
        else:
            x1 = -(path[i-1].x - path[i].x)/ct
            y1 = - (path[i-1].y - path[i].y)/ct
            x2 = (path[i+1].x-path[i].x)/ct
            y2 = (path[i+1].y-path[i].y)/ct
            x = path[i].x+(x1+x2)/ct
            y = path[i].y+(y1+y2)/ct
            b.p1 = Point(x,y)



        if i == len(path)-2:
            x = path[i+1].x -(path[i+1].x-path[i].x)/ct
            y = path[i+1].y-(path[i+1].y-path[i].y)/ct
            b.p2 = Point(x, y)
        else:
            x1 = -(path[i+2].x - path[i+1].x)/ct
            y1 = - (path[i+2].y - path[i+1].y)/ct
            x2 = (path[i].x-path[i+1].x)/ct
            y2 = (path[i].y-path[i+1].y)/ct
            x = path[i+1].x+(x1+x2)/2
            y = path[i+1].y+(y1+y2)/2
            b.p2 = Point(x,y)


        b.p3 = path[i+1]
        b_tmp = b.calc_curve()
        bezier.append(b)
        print(b.max_k())
        plt.plot(b_tmp[0],b_tmp[1])

    plt.axis('equal')
plt.show()


