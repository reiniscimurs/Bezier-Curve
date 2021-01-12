# CubicBezier

Python class for creating quadratic and cubic Bezier curves. Class functions include:

*CubicBezier()/QuadBezier() - creating a Bezier curve
*random() - randomizing Bezier curve control points
*max_k() - calculating maximum curvature
*calc_curve() - calculating points of the curve
*arc_len() - calculating the arc-length of the curve
*add_obstacle() - adding an obstacle
*add_random_obstacle() - adding a random obstacle
*clear() - re-setting all of the curve parameters
*optimization:
    *optimize_k() - minimize maximum curvature of the curve
    *optomize_l() - minimize arc_length of the curve
    *optimize() - simultaniously optimize curvature and the arc-length of the curve
