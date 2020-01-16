# Model Documentation
Implementation details of path planning 

### Summary
Generally, I use finite state machine to implement behavior planning, in which has 3 states (`KL`, `LCL`, `LCR`), 2 cost functions to calculate speed and goal distance cost, and use the `spline` tool to generate trajectories
 for keep lane or change lane behavior, then check collision for every possible trajectory.
 
## Code Explanation
I created 3 new code files:
1. `spline.h` was downloaded from its website without any change
2. `trajectories.h` declares all the functions used for behavior planning and trajectory generation, besides, it declares some structs to encapsulate vehicle info, sensor fusion data, previous path and global map data.
3. `trajectories.cpp` implements the functions defined in `trajectories.h`, and define several consts 

Also modified `main.cpp` to call the functions defined in `trajectories.h` to get the waypoints of next trajectory.

### 1. main.cpp
