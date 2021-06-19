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

It is a partial python implementation of paper:

"Bezier curve-based smoothing for path planner with curvature constraint"

Article available at:
https://ieeexplore.ieee.org/abstract/document/7926545?casa_token=kNNsIfLxILsAAAAA:zJVlg3OF9fZSCfZYK6-dr_ix6RBt_1I-QStlQq3l5GfZyCX3djUArDYiYu_vw9fVslLKmgfFB8E

<p align="left">
<img width=60% src="https://github.com/reiniscimurs/Bezier-Curve/blob/main/smooth_path.png">
</p>
