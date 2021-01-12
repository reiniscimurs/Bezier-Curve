# Quadratic and Cubic Bezier curve

Python class for creating and optimizing quadratic and cubic Bezier curves. Class functions include:

* QuadBezier()/CubicBezier() - creating a Bezier curve
* random() - randomizing Bezier curve control points
* max_k() - calculating maximum curvature
* calc_curve() - calculating points of the curve
* arc_len() - calculating the arc-length of the curve
* add_obstacle() - adding an obstacle
* add_random_obstacle() - adding a random obstacle
* clear() - re-setting all of the curve parameters
* optimization:
    * optimize_k() - minimize maximum curvature of the curve
    * optimize_l() - minimize arc_length of the curve
    * optimize() - simultaniously optimize curvature and the arc-length of the curve
    
    
<p align="left">
<img width=60% src="https://github.com/reiniscimurs/Bezier-Curve/blob/main/Bezier.PNG">
</p>
