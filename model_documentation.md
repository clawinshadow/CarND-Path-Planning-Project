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
Define 2 global variables: 

1. `state` to record the current state in FSM, which initial state is __`KL`__ 
2. `lane` to track the current lane of the ego car, which initial value is __1__

Then in the callback function `onMessage(...)`:

1. Package various data for behavior plannning and trajectory calculation, listed as below
   * `VehiclePose` includes all the vehicle info of ego car, such as coordinates, yaw and speed, it's defined in `trajectories.h`
   * `PreviousPath` includes the previous path data given to the planner, also defined in `trajectories.h`
   * `Map` is the global map data, defined in `trajectories.h`
   
2. Call the `get_trajectory()` function to get the next trajectory, which contains the main logic of this project, is implemented in `trajectories.cpp`

3. Update `state` and `lane` variables and pass next waypoints to simulator  

### 2. trajectories.cpp

We start from `get_trajectory()`, at the beginning I call `successor_states()` to get a list of next possible states

   * `successor_states()`: for this finite state machine, I only retain 3 states - __KL/LCL/LCR__, KL can leads to KL, LCL or LCR or both, but LCL or LCR can only leads to KL. On the other hand, I add some logic to check if the ego car is changing lane, it's necessary to ensure the completeness of LCL/LCR action, if it's changing lane left or right, the next state will always be __KL__ until it's on the target lane.
   
Then we loop the list of next possible states, calculate the trajectory for each state,the cost for each trajectory, sort them to get the trajectory with a minimum cost. 

I define a struct `Trajectory` to encapsulate all the data for generating waypoints and calculating cost, in which `target_s, target_d, target_velocity` for generating waypoints, `final_s, final_speed` for calculating cost, `waypoints` to store the waypoints

   * `keep_lane_trajectory()`: in KL state, the `target_d` keep the same as current lane, `target_s` should always be the end s of previous path plus 30, that's straighforward. Then I check if there is a car ahead the ego car in current lane, if exists, then the `final_s` and `final_speed` should be the same as this car ahead. What's more, if it's too close to the car ahead, we need to decrease speed gradually to the speed of car ahead to avoid collision. 
   * `change_lane_trajectory()`: if we want to change lane, the `target_d` definitely should be the d value of intended lane, `target_s` is still the end s of previous path plus 30.  For `final_s` and `final_speed`, we also use sensor fusion data to find if there is a car ahead in the intended lane, I don't change the ref velocity when changing lane, because it will soon switch to __KL__ state in next cycle, it's enough for __KL__ state to control the speed.
   * `get_adjacent_car()`: use sensor fusion data to get the closest car behind/ahead the ego for a specific lane.
   * `generate_waypoints()`: use `target_s, target_d` and the end points of previous path to generate anchor points, then use `spline` tool and `target_velocity` to calculate interpolated points, then mix them with previous path to generate the next waypoints.
