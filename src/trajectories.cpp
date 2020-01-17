#include "trajectories.h"
#include "spline.h"

#include <map>
#include <iostream>

using namespace std;

double ref_velocity = 0;

const int LANES_AVAILABLE = 3;
const double MAX_VELOCITY = 48.5; //mph
const double EFFICIENCY_WEIGHT = 0.7; //weights of inefficiency cost
const double REACH_GOAL_WEIGHT = 0.3; //weights of goal distance cost

const double POSSIBLE_COLLISION_DISTANCE = 25;

int get_lane(const double &_d)
{
  return floor(_d / 4);
}

double inefficiency_cost(const double &_target_speed, const double &_intented_speed, const double &_curr_speed) 
{
  return 0.5 * (2.0 * _target_speed - _intented_speed - _curr_speed) / _target_speed;
}

double goal_distance_cost(double delta_d, double delta_s) 
{
  return (1 - exp(-(std::abs(delta_d) / delta_s)));
}

bool verify_waypoints(const vector<vector<double>> &_waypoints,
                      const VehiclePose &_adjacent_car,
                      const Map &_map)
{
  bool valid_path = true;
  
  vector<double> x_vals = _waypoints[0];
  vector<double> y_vals = _waypoints[1];

  for (int i = 0; i < x_vals.size(); i++)
  {
    //coordinate of current waypoint of ego car
    double self_x = x_vals[i];
    double self_y = y_vals[i];

    //prediction of Frenet coordinates of the car in the target lane
    double d_future = _adjacent_car.d; 							        // I suppose the other cars all keep their lanes 
    double s_future = _adjacent_car.s + 0.02 * i * _adjacent_car.speed; // I suppose the adjacent car's speed will not change
    //convert to xy coordinates
    vector<double> car_adjacent_future = getXY(s_future, d_future, _map.map_waypoints_s, _map.map_waypoints_x, _map.map_waypoints_y);

    double gap = distance(self_x, self_y, car_adjacent_future[0], car_adjacent_future[1]);
    if (gap < POSSIBLE_COLLISION_DISTANCE)
    {
      valid_path = false; //will collide with adjacent car
      break;
    }
  }
  
  return valid_path;
}

bool check_collision(const Trajectory &_trajectory, 
                     const VehiclePose &_curr_pose, 
                     const vector<vector<double>> &_sensor_fusion,
                     const Map &_map)
{
  int curr_lane = get_lane(_curr_pose.d);
  int target_lane = get_lane(_trajectory.target_d);
  if (target_lane != curr_lane)
  {
    //check car behind main car in the target lane
  	VehiclePose car_behind;
  	if (get_car_adjacent(target_lane, _sensor_fusion, _curr_pose, false, car_behind))
    {
      if (!verify_waypoints(_trajectory.waypoints, car_behind, _map))
      {
        std::cout << "Potential collision detected.. BEHIND" << std::endl; 
        return true;
      }
    }
    //check car ahead main car in the target lane
    VehiclePose car_ahead;
    if (get_car_adjacent(target_lane, _sensor_fusion, _curr_pose, true, car_ahead))
    {
      if (!verify_waypoints(_trajectory.waypoints, car_ahead, _map))
      {
        std::cout << "Potential collision detected.. AHEAD" << std::endl;
        return true;
      }
    }    
  }
  
  return false;
}

double calculate_cost(const Trajectory &_trajectory, 
                      const VehiclePose &_curr_pose, 
                      const vector<vector<double>> &_sensor_fusion,
                      const Map &_map)
{
  double ineffi_cost = inefficiency_cost(MAX_VELOCITY, _trajectory.final_speed, _curr_pose.speed);
  
  double delta_d = get_lane(_trajectory.target_d) - get_lane(_curr_pose.d); //maybe it should be the s/d of prev path
  double delta_s = _trajectory.final_s - _curr_pose.s;
  double distance_cost = goal_distance_cost(delta_d, delta_s);
  //debug
  //printf("inefficiency cost: %f, distance_cost: %f \n", ineffi_cost, distance_cost);
  
  //check collision
  bool collision = check_collision(_trajectory, _curr_pose, _sensor_fusion, _map);
  if (collision)
  {
    //std::cout << "Potential Collision Detected..\n";
    return 1;  //collision leads to a max cost
  }
  
  double cost = EFFICIENCY_WEIGHT * ineffi_cost + REACH_GOAL_WEIGHT * distance_cost;
  return cost;
}

Trajectory get_trajectory(const string &_state, 
                          const int &_intended_lane,
                          const VehiclePose _curr_pose,
                          const vector<vector<double>> &_sensor_fusion, 
                          const PreviousPath &_prev,
                          const Map &_map)
{
  Trajectory result;
  
  vector<string> potential_states = successor_states(_intended_lane, _curr_pose, _state);
  
  map<double, Trajectory> cost_map;
  for (int i = 0; i < potential_states.size(); i++)
  {
    string next_state = potential_states[i];
    //generate trajectory
    Trajectory traj;
    int target_lane;
    if (next_state.compare("KL") == 0)
    {
      traj = keep_lane_trajectory(_intended_lane, _sensor_fusion, _prev, _curr_pose, _map);
      traj.state = "KL";
    }
    else if (next_state.compare("LCL") == 0)
    {
      target_lane = _intended_lane - 1;
      traj = change_lane_trajectory(target_lane, _sensor_fusion, _prev, _curr_pose, _map);
      traj.state = "LCL";
    }
    else if (next_state.compare("LCR") == 0)
    {
      target_lane = _intended_lane + 1;
      traj = change_lane_trajectory(target_lane, _sensor_fusion, _prev, _curr_pose, _map);
      traj.state = "LCR";
    }
    else
    {
      std::cerr << "INVALID SUCCESSOR STATE : " << next_state << std::endl;
      continue;
    }
      
    double cost = calculate_cost(traj, _curr_pose, _sensor_fusion, _map);
    cost_map[cost] = traj;
  }
  
  if (cost_map.size() < 1)
  {
    std::cerr << "Empty cost map" << std::endl;
    return result;
  }
  
  //debug
  printf("--------------cost map-----------------\n");
  for (auto it = cost_map.cbegin(); it != cost_map.cend(); it++)
  {
    int target_lane = get_lane((it->second).target_d);
    printf("possible state: %s, intended lane: %d, target velocity: %f, cost: %f\n", 
           ((it->second).state).c_str(), target_lane, (it->second).target_velocity, it->first);
  }
  
  result = cost_map.cbegin()->second;
  return result;
}

vector<string> successor_states(int _lane, const VehiclePose _curr_pose, const std::string &_state)
{
  vector<string> states;
  states.push_back("KL");
  //
  double target_d = 2 + _lane * 4;
  if (abs(target_d - _curr_pose.d) > 0.5)
  {
    std::cout << "Changing lane.. " << std::endl;
    return states;
  }
  if(_state.compare("KL") == 0) {
    if (_lane != 0)
    	states.push_back("LCL");
    if (_lane != LANES_AVAILABLE - 1)
    	states.push_back("LCR");
  } 
  
  // If state is "LCL" or "LCR", then just return "KL"
  return states;
}

vector<vector<double>> generate_waypoints(const double &_s, 
                                          const double &_d, 
                                          const double &_velocity,
                                          const VehiclePose &_curr_pose,
                                          const PreviousPath &_prev,
                                          const Map &_map)
{
  int prev_size = _prev.xs.size();
          
  vector<double> next_x_vals;
  vector<double> next_y_vals;
  
  vector<double> pts_x;
  vector<double> pts_y;

  double ref_x = _curr_pose.x;
  double ref_y = _curr_pose.y;
  double ref_yaw = deg2rad(_curr_pose.yaw);

  if (prev_size < 2)
  {
    double prev_car_x = _curr_pose.x - cos(_curr_pose.yaw);
    double prev_car_y = _curr_pose.y - sin(_curr_pose.yaw);

    pts_x.push_back(prev_car_x);
    pts_x.push_back(_curr_pose.x);

    pts_y.push_back(prev_car_y);
    pts_y.push_back(_curr_pose.y);
  }
  else
  {
    ref_x = _prev.xs[prev_size - 1];
    ref_y = _prev.ys[prev_size - 1];

    double ref_x_prev = _prev.xs[prev_size - 2];
    double ref_y_prev = _prev.ys[prev_size - 2];

    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    pts_x.push_back(ref_x_prev);
    pts_x.push_back(ref_x);

    pts_y.push_back(ref_y_prev);
    pts_y.push_back(ref_y);
  }

  vector<double> next_wp_0 = getXY(_s, _d, _map.map_waypoints_s, _map.map_waypoints_x, _map.map_waypoints_y);
  vector<double> next_wp_1 = getXY(_s+30, _d, _map.map_waypoints_s, _map.map_waypoints_x, _map.map_waypoints_y);
  vector<double> next_wp_2 = getXY(_s+60, _d, _map.map_waypoints_s, _map.map_waypoints_x, _map.map_waypoints_y);

  pts_x.push_back(next_wp_0[0]);
  pts_x.push_back(next_wp_1[0]);
  pts_x.push_back(next_wp_2[0]);

  pts_y.push_back(next_wp_0[1]);
  pts_y.push_back(next_wp_1[1]);
  pts_y.push_back(next_wp_2[1]);

  // transform to base_link frame
  for (int i = 0; i < pts_x.size(); i++)
  {
    double shift_x = pts_x[i] - ref_x;
    double shift_y = pts_y[i] - ref_y;

    pts_x[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
    pts_y[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
    
    //debug
    //printf("anchor points %d: x: %f, y: %f\n", i, pts_x[i], pts_y[i]);
  }

  tk::spline s;
  s.set_points(pts_x ,pts_y);

  for (int i = 0; i < prev_size; i++)
  {
    next_x_vals.push_back(_prev.xs[i]);
    next_y_vals.push_back(_prev.ys[i]);
  }

  double target_x = 30;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x + target_y * target_y);

  double x_add_on = 0;

  for (int i = 0; i < 50 - prev_size; i++)
  {
    double N = target_dist / (0.02 * _velocity / 2.24);
    double x_point = x_add_on + target_x / N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);            
  }
  
  vector<vector<double>> waypoints {next_x_vals, next_y_vals};
  return waypoints;
}

//get the closest car ahead or behind the ego car.
bool get_car_adjacent(const int &_lane, 
                      const vector<vector<double>> &_sensor_fusion, 
                      const VehiclePose &_ego_car,
                      bool is_ahead,
                      VehiclePose &_car_ahead)
{
  if (_sensor_fusion.size() < 1)
    return false;
  
  map<float, VehiclePose> cars_ahead;
  for (int i = 0; i < _sensor_fusion.size(); i++)
  {
    float d = _sensor_fusion[i][6];
    if (d > (2+4*_lane+2) || d < (2+4*_lane-2))
      continue;
    
    double s = _sensor_fusion[i][5];
    if (is_ahead)
    {
      if (s < _ego_car.s)
      	continue;
    }
    else
    {
      if (s >= _ego_car.s)
        continue;
    }
    
    double vx = _sensor_fusion[i][3];
    double vy = _sensor_fusion[i][4];
    double speed = sqrt(vx*vx + vy*vy);
    double x = _sensor_fusion[i][1];
    double y = _sensor_fusion[i][2];
    
      
    VehiclePose car(x, y, s, d, 0.0, speed);
    cars_ahead[s] = car;
  }
  
  if (cars_ahead.size() == 0)
    return false;

  _car_ahead = cars_ahead.cbegin()->second;  
  return true;
}

Trajectory keep_lane_trajectory(const int &_lane,
                                const vector<vector<double>> &_sensor_fusion, 
                                const PreviousPath &_prev,
                                const VehiclePose &_curr_pose,
                                const Map &_map)
{
  Trajectory result;
  if (_prev.xs.size() < 1)
    result.target_s = _curr_pose.s +30; //for the initial state
  else
  	result.target_s = _prev.end_s + 30;
  result.target_d = 2 + 4 * _lane;
  
  bool too_close = false;
  
  VehiclePose car_ahead;
  if (get_car_adjacent(_lane, _sensor_fusion, _curr_pose, true, car_ahead))
  {
    double check_car_s = car_ahead.s + car_ahead.speed * 0.02 * _prev.xs.size();
    if (check_car_s > _prev.end_s && (check_car_s - _prev.end_s) < 20)
    {
      //debug
      //printf("keep lane - car_ahead_s: %f, check_car_s: %f, prev_end_s: %f \n", car_ahead.s, check_car_s, _prev.end_s);
      too_close = true;
    }
    result.final_s = car_ahead.s;
    result.final_speed = car_ahead.speed * 2.24; 
  }
  else
  {
    result.final_s = result.target_s + 300;
    result.final_speed = MAX_VELOCITY;
  }
  
  if (too_close)
  {
    // if too close, then brake the car, decrease speed to the same as the car ahead
    if (ref_velocity > car_ahead.speed)
      ref_velocity = ref_velocity - 0.224;
    else
      ref_velocity = car_ahead.speed;
  }
  else
  {
    if (ref_velocity < MAX_VELOCITY)
      ref_velocity = ref_velocity + 0.224;
  }
  
  //debug
  printf("KL - final_s: %f, target_s: %f, final_speed: %f\n", result.final_s, result.target_s, result.final_speed);

  result.target_velocity = ref_velocity;
  
  //debug
//   printf("keep lane - result.target_d : %f, target_s: %f, target_velocity: %f, curr speed: %f\n", 
//          result.target_d, result.target_s, result.target_velocity, _curr_pose.speed);
  result.waypoints = generate_waypoints(result.target_s, result.target_d, result.target_velocity, _curr_pose, _prev, _map);
  return result;
}

Trajectory change_lane_trajectory(const int &_lane, 
                                  const vector<vector<double>> &_sensor_fusion, 
                                  const PreviousPath &_prev,
                                  const VehiclePose &_curr_pose,
                                  const Map &_map)
{
  Trajectory result;
  result.target_d = 2 + 4 * _lane;
  if (_prev.xs.size() < 1)
    result.target_s = _curr_pose.s + 30; //for the initial state
  else
  	result.target_s = _prev.end_s + 30;
  
//   int curr_lane = get_lane(_curr_pose.d);
//   VehiclePose car_ahead_current_lane;
//   if (get_car_adjacent(curr_lane, _sensor_fusion, _curr_pose, true, car_ahead_current_lane))
//   {
//     double check_car_s = car_ahead_current_lane.s + car_ahead_current_lane.speed * 0.02 * _prev.xs.size();
//     if (check_car_s > _prev.end_s && (check_car_s - _prev.end_s) < 30)
//     {
//       //debug
//       //printf("change lane - car_ahead_s: %f, check_car_s: %f, prev_end_s: %f \n", car_ahead_current_lane.s, check_car_s, _prev.end_s);
//       result.target_s = check_car_s;  //avoid collide with the car ahead when changing lane
//     }
//   }
  
  VehiclePose car_ahead_intended_lane;
  if (get_car_adjacent(_lane, _sensor_fusion, _curr_pose, true, car_ahead_intended_lane))
  {
    result.final_s = car_ahead_intended_lane.s + car_ahead_intended_lane.speed * 0.02 * _prev.xs.size();
    result.final_speed = car_ahead_intended_lane.speed * 2.24;
  }
  else
  {
    result.final_s = result.target_s + 300;
    result.final_speed = MAX_VELOCITY;
  }
  //debug
  printf("CL - final_s: %f, target_s: %f, final_speed: %f\n", result.final_s, result.target_s, result.final_speed);
  
  result.target_velocity = ref_velocity; // ref_velocity already +0.448 in KL state
  //debug
//   printf("change lane - result.target_d : %f, target_s: %f, target_velocity: %f, curr speed: %f\n", 
//          result.target_d, result.target_s, result.target_velocity, _curr_pose.speed);
  result.waypoints = generate_waypoints(result.target_s, result.target_d, result.target_velocity, _curr_pose, _prev, _map);
  return result;
}
