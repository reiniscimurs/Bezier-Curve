from CubBezier import CubicBezier

print("Creating a Bezier curve object")
curve = CubicBezier(p1x=-1, p1y=0, p2x=-0.5, p2y=0.5, p3x=0.5, p3y=-0.5, p4x=1, p4y=0)
curve.plot()

print("Adding an obstacle")
curve.add_obstacle(x=0, y= 0, radius= 0.3)
curve.plot()

print("Adding an additional random obstacle")
curve.add_random_obstacle(min_x=-1, max_x=1, min_y=-0.5, max_y=0.5, min_radius=0.05, max_radius=0.5)
curve.plot()

print("Optimizing arc-length")
curve.optimize_l()
curve.plot()

print("Optimizing curvature")
curve.optimize_k()
curve.plot()

print("Simultaniously optimizing arc-length and curvature")
curve.optimize()
curve.plot()

print("Re-initializing the Bezier curve")
curve.clear()

print("Creating a random Bezier curve")
curve.random(min=-1,max=1)
curve.plot()

print("Adding random obstacles")
for i in range(0,3):
    curve.add_random_obstacle(min_x=min(curve.p1.x,curve.p4.x), max_x=max(curve.p1.x,curve.p4.x),
                              min_y=min(curve.p1.y,curve.p4.y), max_y=max(curve.p1.y,curve.p4.y),
                              min_radius=0.05, max_radius=0.5)
curve.plot()

print("Simultaniously optimizing arc-length and curvature")
curve.optimize()
curve.plot()