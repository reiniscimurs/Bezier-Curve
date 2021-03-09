from Bezier import CubicBezier, QuadBezier, Point
import matplotlib.pyplot as plt
from scipy.optimize import minimize, rosen, rosen_der, Bounds
import copy
import math
import random


def check_obst(x1, y1, x2, y2, ox, oy):
    """Check the distance of intersection between a line from (x1,y1) to (x2,y2) and a point (ox,oy).
     The point represents the origin of the obstacle."""
    a = y1 - y2
    b = x2 - x1
    c = (x1-x2)*y1 + x1*(y2-y1)
    dist = ((abs(a * ox + b * oy + c))/math.sqrt(a * a + b * b))
    return dist

def calc_p1(p,p_p,p_m, i, cd):
    """Calculate the control point p1 of the current cubic Bezier curve."""
    if not i:
        x = p.x + ((p_p.x - p.x) / cd)
        y = p.y + ((p_p.y - p.y) / cd)
    else:
        x1 = -(p_m.x - p.x) / cd
        y1 = - (p_m.y - p.y) / cd
        x2 = (p_p.x - p.x) / cd
        y2 = (p_p.y - p.y) / cd
        x = p.x + (x1 + x2) / cd
        y = p.y + (y1 + y2) / cd
    return x, y

def calc_p2(p, p_p, p_pp, i, cd):
    """Calculate the control point p2 of the current cubic Bezier curve."""
    x1 = -(p_pp.x - p_p.x) / cd
    y1 = - (p_pp.y - p_p.y) / cd
    x2 = (p.x - p_p.x) / cd
    y2 = (p.y - p_p.y) / cd
    x = p_p.x + (x1 + x2) / cd
    y = p_p.y + (y1 + y2) / cd
    return x, y

def optimizer_p(cd, path, i, obs, path_penalty):
    """Optimizer of the current path. Reduce the piece-wise path length in the free space of the environment."""
    p_tmp = copy.deepcopy(path)
    p_tmp[i].x = p_tmp[i].x + cd[0]
    p_tmp[i].y = p_tmp[i].y + cd[1]
    r1 = math.sqrt((p_tmp[i-1].x - p_tmp[i].x)**2+(p_tmp[i-1].y - p_tmp[i].y)**2)
    r2 = math.sqrt((p_tmp[i+1].x - p_tmp[i].x)**2+(p_tmp[i+1].y - p_tmp[i].y)**2)
    penalty1 = 0
    penalty2 = 0
    if obstacles:
        for o in obs:
            d1 = check_obst(p_tmp[i-1].x, p_tmp[i-1].y, p_tmp[i].x, p_tmp[i].y, o[0].x, o[0].y)
            if d1< o[1]:
                penalty1 = max(penalty1,(o[1] - d1)*path_penalty)
            d2 = check_obst(p_tmp[i].x, p_tmp[i].y, p_tmp[i+1].x, p_tmp[i+1].y, o[0].x, o[0].y)
            if d2 < o[1]:
                penalty2 = max(penalty1,(o[1] - d1)*path_penalty)
    return  r1 + r2 + abs(r1-r2) + penalty1 + penalty2

def optimizer_k(cd, k, path, i, obs, curve_penalty_multiplier, curve_penalty_divider, curve_penalty_obst):
    """Bezier curve optimizer that optimizes the curvature and path length by changing the distance of p1 and p2 from
     points p0 and p3, respectively. """
    p_tmp = copy.deepcopy(path)
    if i+3 > len(path)-1:
        b = CubicBezier()
        b.p0 = p_tmp[i]
        x, y = calc_p1(p_tmp[i], p_tmp[i + 1], p_tmp[i - 1], i, cd[0])
        b.p1 = Point(x, y)
        x, y = calc_p2(p_tmp[i-1], p_tmp[i + 0], p_tmp[i + 1], i, cd[1])
        b.p2 = Point(x, y)
        b.p3 = p_tmp[i + 1]
        B = CubicBezier()
    else:
        b = CubicBezier()
        b.p0 = p_tmp[i]
        x, y = calc_p1(p_tmp[i],p_tmp[i+1],p_tmp[i-1], i, cd[0])
        b.p1 = Point(x, y)
        x, y = calc_p2(p_tmp[i],p_tmp[i+1],p_tmp[i+2], i, cd[1])
        b.p2 = Point(x, y)
        b.p3 = p_tmp[i + 1]
        B = CubicBezier()
        B.p0 = p_tmp[i]
        x, y = calc_p1(p_tmp[i+1], p_tmp[i + 2], p_tmp[i], i, 10)
        B.p1 = Point(x, y)
        x, y = calc_p2(p_tmp[i+1], p_tmp[i + 2], p_tmp[i + 3], i, 10)
        B.p2 = Point(x, y)
        B.p3 = p_tmp[i + 1]

    m_k = b.max_k()
    if m_k>k:
        m_k= m_k*curve_penalty_multiplier
    else:
        m_k = m_k/curve_penalty_divider

    f = lambda x, y: max(math.sqrt((x[0] - y[0].x) ** 2 + (x[1] - y[0].y) ** 2) * curve_penalty_obst, 10) if math.sqrt(
        (x[0] - y[0].x) ** 2 + (x[1] - y[0].y) ** 2) < y[1] else 0
    b_t = b.calc_curve(granuality=10)
    b_t = zip(b_t[0],b_t[1])
    B_t = B.calc_curve(granuality=10)
    B_t = zip(B_t[0], B_t[1])
    penalty1 = 0
    penalty2 = 0
    for o in obs:
        for t in b_t:
            penalty1 = max(penalty1,f(t,o))
        for t in B_t:
            penalty2 = max(penalty2,f(t,o))
    return b.arc_len(granuality=10)+B.arc_len(granuality=10)+m_k + penalty1 + penalty2

"""Parameters for calculation"""
nr_of_points = 10 # Number o points in the path
max_k = 20 # Ideal maximum curvature (not guaranteed)
obstacles = True # Weather to use obstacles
n_path_opt = 5 # Number of optimization cycles
path_penalty = 1000 # Penalty for path optimizer
curve_penalty_multiplier = 1000 # Penalty multiplier for breaking maximum curvature limit
curve_penalty_divider = 10 # Discount divider for being under the maximum curvature limit
curve_penalty_obst = 10000 # Penalty multiplier for collision with obstacles

path = []
bez = []
obs = []

#Create a random path
for i in range(nr_of_points):
    p = Point(random.uniform(i,i+1),random.uniform(0,i))
    path.append(p)
    b = CubicBezier()
    bez.append(b)

#If obstacles are enabled, add obstacles to the environment
if obstacles:
    for i in range(nr_of_points):
        f = False
        while not f:
            x = random.uniform(i, i + 2)
            y = random.uniform(0, i)
            radius = random.uniform(0, 1)
            d = 1000
            for p_i in range(1, len(path)):
                d = min(d, check_obst(path[p_i-1].x,path[p_i-1].y,path[p_i].x,path[p_i].y,x,y))
            if d>radius:
                f = True
        o = Point(x, y)
        obs.append([o, radius])

# Plot the initial path and the obstacles
xs = [x.x for x in path]
ys = [y.y for y in path]
for i in range(len(obs)):
    plt.gcf().gca().add_artist(
        plt.Circle((obs[i][0].x, obs[i][0].y), obs[i][1], color='r'))
plt.axis('equal')
plt.plot(xs, ys)
plt.show()

# Optimize the initial path for n_path_opt cycles
for m in range(n_path_opt):
    if m%2:
        for i in range(1,len(path)-1):
                x0 = [0.0, 0.0]
                bounds = Bounds([-1, -1], [1, 1])
                res = minimize(optimizer_p, x0, args=(path, i, obs, path_penalty), method='TNC', tol=1e-7, bounds=bounds)
                x, y = res.x
                path[i].x += x
                path[i].y += y
    else:
        for i in range(len(path)-1,1):
                x0 = [0.0, 0.0]
                bounds = Bounds([-1, -1], [1, 1])
                res = minimize(optimizer_p, x0, args=(path, i, obs, path_penalty), method='TNC', tol=1e-7, bounds=bounds)
                x, y = res.x
                path[i].x += x
                path[i].y += y

bezier = []
bx = []
by = []
plt.axis('equal')
plt.plot(xs, ys)

# Create Bezier curves and optimize them
for i in range(len(path)-1):
        bez[i].p0 = path[i]
        bounds = Bounds([0.1, 0.1], [100, 100])
        x0 = [4.0, 4.0]
        res = minimize(optimizer_k, x0, args=(max_k, path, i, obs, curve_penalty_multiplier, curve_penalty_divider,
                                              curve_penalty_obst), method='TNC', tol=1e-8, bounds=bounds)
        ct1, ct2 = res.x
        x, y = calc_p1(path[i], path[i + 1], path[i-1], i, ct1)
        bez[i].p1 = Point(x, y)
        try:
            x, y = calc_p2(path[i], path[i+1],path[i+2], i, ct2)
        except:
            x, y = calc_p2(path[i-1], path[i], path[i + 1], i, ct2)
        bez[i].p2 = Point(x,y)
        bez[i].p3 = path[i+1]
        print(f'{i}: Optimizer output:{res.x}, Max curvature of the Curve: { bez[i].max_k()}')
        b_tmp = bez[i].calc_curve()
        bezier.append(bez[i])
        bx +=b_tmp[0]
        by += b_tmp[1]

# Plot the output
plt.plot(bx,by)
xs = [x.x for x in path]
ys = [y.y for y in path]
for i in range(len(obs)):
    plt.gcf().gca().add_artist(
        plt.Circle((obs[i][0].x, obs[i][0].y), obs[i][1], color='r'))
plt.axis('equal')
plt.plot(xs, ys)
plt.show()


