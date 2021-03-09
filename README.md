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

# Curvature constrained path smoothing

The path_smoothing.py contains a curvature constrained path smoothing algorithm. A path is obtained between random obstacles in the environment. Initial path is optimized by employing an optimization function. Afterwards, Bezier curve is obtained between the path points and optimized to minimize the path length while maintaining the maximum curvature. (Currently in development and does not guarantee collision free path).

It is a partial python implementation of:
@inproceedings{cimurs2017bezier,
  title={Bezier curve-based smoothing for path planner with curvature constraint},
  author={Cimurs, Reinis and Hwang, Jaepyung and Suh, Il Hong},
  booktitle={2017 First IEEE International Conference on Robotic Computing (IRC)},
  pages={241--248},
  year={2017},
  organization={IEEE}
}
