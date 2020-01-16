#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

#include <vector>
#include <string>
#include "helpers.h"

using namespace std;

struct VehiclePose
{
  double x;
  double y;
  double s;
  double d;
  double yaw; 
  double speed;
  
  VehiclePose(double _x, double _y, double _s, double _d, double _yaw, double _speed)
  {
    x = _x;
    y = _y;
    s = _s;
    d = _d;
    yaw = _yaw;
    speed = _speed;
  }
  
  VehiclePose()
  {
    VehiclePose(0, 0, 0, 0, 0, 0);
  }
};

struct PreviousPath
{
  vector<double> xs;
  vector<double> ys;
  double end_s;
  double end_d;
  
  PreviousPath(double _s, double _d, const vector<double> &_xs, const vector<double> &_ys)
  {
    end_s = _s;
    end_d = _d;
    xs = _xs;
    ys = _ys;
  }
};

struct Trajectory
{
  string state;
  double target_s;        // using for generate waypoints
  double target_d;        // using for generate waypoints, and goal_distance_cost
  double target_velocity; // using for generate waypoints

  double final_speed;     // using for inefficiency cost, the max velocity car intended to reach
  double final_s;         // using for goal_distance_cost
  vector<vector<double>> waypoints;
};

struct Map
{
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  
  Map(const vector<double> &_x, const vector<double> &_y, const vector<double> &_s)
  {
    map_waypoints_x = _x;
    map_waypoints_y = _y;
    map_waypoints_s = _s;
  }
};

int get_lane(const double &_d);

double goal_distance_cost(double delta_d, double delta_s);
double inefficiency_cost(const double &_target_speed, const double &_intented_speed, const double &_curr_speed); 
double calculate_cost(const Trajectory &_trajectory, 
                      const VehiclePose &_curr_pose, 
                      const vector<vector<double>> &_sensor_fusion,
                      const Map &_map);

bool verify_waypoints(const vector<vector<double>> &_waypoints,
                      const VehiclePose &_adjacent_car,
                      const Map &_map);
bool check_collision(const Trajectory &_trajectory, 
                      const VehiclePose &_curr_pose, 
                      const vector<vector<double>> &_sensor_fusion,
                      const Map &_map);
  
vector<string> successor_states(int _lane, const VehiclePose _curr_pose, const std::string &_state);

Trajectory get_trajectory(const string &_state, 
                          const int &_intended_lane,
                          const VehiclePose _curr_pose,
                          const vector<vector<double>> &_sensor_fusion, 
                          const PreviousPath &_prev,
                          const Map &_map);

vector<vector<double>> generate_waypoints(const double &_s, 
                                          const double &_d, 
                                          const double &_velocity,
                                          const VehiclePose &_curr_pose,
                                          const PreviousPath &_prev,
                                          const Map &_map);

Trajectory keep_lane_trajectory(const int &_lane,
                                const vector<vector<double>> &_sensor_fusion, 
                                const PreviousPath &_prev,
                                const VehiclePose &_curr_pose,
                                const Map &_map);

Trajectory change_lane_trajectory(const int &_lane, 
                                  const vector<vector<double>> &_sensor_fusion, 
                                  const PreviousPath &_prev,
                                  const VehiclePose &_curr_pose,
                                  const Map &_map);

bool get_car_adjacent(const int &_lane, 
                      const vector<vector<double>> &_sensor_fusion, 
                      const VehiclePose &_ego_car,
                      bool is_ahead,
                      VehiclePose &_car_ahead);
#endif