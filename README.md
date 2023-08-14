# MATLAB-Implementation-of-Pure_Pursuit-and-Vector-Field-Histogram-

In this project, Robotics simulation toolbox in MATLAB was used to implement path following of a turtlebot in presence of obstacles from a start to goal location. The path following task was achieved by implementing **PURE PURSUIT** and obstacle avoidance was achived by implementing **VECTOR FIELD HISTOGRAM (VFH)**.

For full details on implementation see this [report](https://github.com/adarsh2798/MATLAB-Implementation-of-Pure_Pursuit-and-Vector-Field-Histogram-/blob/main/EXP1_GITHUB/ee615_exp1%20_REPORT_both.pdf)

## PURE PURSUIT

Pure pursuit alogrithm takes a (N x 2) matrix of 'N' waypoints and calculates the curvature required to drive the robot to the "lookahead" point along the line joining the current and next waypoint. It calculates the angualr velocity with which the robot needs to move with respect to ICR (instantaneous center of rotation).
