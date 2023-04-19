# Installation
This package is intended for ROS (tested with Noetic). You need to clone my ROSTools package, install dependencies and OpenMP and GSL:

```
https://github.com/oscardegroot/ros_tools.git
rosdep install --from-paths src --ignore-src -r -y
sudo apt-get install libgsl-dev libomp-dev
```

Then build the workspace:

```
catkin build guidance_planner
```


# Running
For an example on how to use the planner, see `src/example.cpp`. 

To run the example:
```
roslaunch guidance_planner example.launch
```

You should see the following:
![Example](docs/example.png)

For profiling information (i.e., computation times) go to `chrome://tracing/` in google chrome. Then click `load` and select `guidance_planner/profiler.json`. This should show some timings:
![Timing](docs/timing.png)