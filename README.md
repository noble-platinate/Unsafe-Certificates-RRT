# A Comparative Study of Collision-Checking in Sampling-Based Planners
## Introduction
This is a Python package which demonstrates the difference in performance between collision-checking algorithms in the sampling-based planning algorithm RRT.
## Setting up the package
```
git clone https://github.com/noble-platinate/Unsafe-Certificates-RRT.git
```
The config file can be used to switch and tune parameters such as step_size, search_radius, etc.
## Installing Dependencies
```
pip3 install -r requirements.txt
pip3 install opencv-python
```
## Running RRT
```
cd individual
python3 rrt.py
```

## Simulation

https://user-images.githubusercontent.com/64150934/214223041-3ade6d13-b487-4ff3-9dec-66a204539678.mp4

## Sample Results
<p float="left">
    <img src="media/normal.png" width = "265" height = "265">
    <img src="media/rrt_safe.png" width = "265" height = "265">
    <img src="media/rrt_unsafe.png" width = "265" height = "265">
</p>
(left to right) RRT with normal collision checking, RRT with safety certificates, RRT with safety+unsafety certificates

### Comparison Results
<p float="left">
    <img src="media/all.png" width = "400" height = "370"><br>
    Red line: Normal RRT, Orange line: RRT with safety certificates, Blue line: RRT with safety+unsafety certificates, Green line: RRT with safety+unsafety certificates level 2
    <br><br>
</p>
Above are the results for a run on a randomly generated 1000x1000 obstacle map, step size = 12px, and search radius = 17px. It demonstrates number of iterations vs total time taken.
