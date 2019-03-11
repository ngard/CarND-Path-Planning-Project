#pragma once

#include <vector>
#include <string>
#include "spline.h"

typedef struct Point {
  double x;
  double y;
} Point;

using RelativeVelocity = Point;

using ObstacleInfo = std::vector<double>;

class PathPlanner {
public:
  enum State {
	      INIT,
	      KEEP_LANE,
	      CHANGE_LANE_LEFT,
	      CHANGE_LANE_RIGHT
  };

private:
  double x, y, s, d, yaw;
  double speed; // [m/s]
  int current_lane;
  double cos_yaw, sin_yaw;
  tk::spline spline_path, spline_speed;
  std::vector<double> path_x, path_y;
  double start_speed; // [m/s] starting speed to generate smooth path
  double path_point_x;
  std::vector<ObstacleInfo> sensor_fusion;
  double min_time_to_collision, min_distance_to_obstacle; // time to collision ahead of the vehicle to controll speed.
  double ttc_left_lane, ttc_current_lane, ttc_right_lane; // time to collision in each lane
  double clear_distance_left_lane, clear_distance_current_lane, clear_distance_right_lane; // anterior distance without obstacles in each lane
  bool do_not_go_right, do_not_go_left; // flags to avoid lateral collision while changing lane
  static int cost_keep_lane, cost_change_lane_to_right, cost_change_lane_to_left; // cost to decide whether keeping lane or changing lane

private:
  static std::vector<double> waypoints_x, waypoints_y, waypoints_s; // maps
  static double target_speed; // [mph]
  static State state;
  static int target_lane;
  static int keep_lane_count; // counter of the length of KEEP_LANE

  constexpr static double max_speed = 49.0; // [mph]
  // minimum acceptable length to excersize lane change after keeping lane
  constexpr static int kMinimumKeepLaneCount = 50*5;
  // inherit costs from previous cycle to avoid frequent state change
  constexpr static double kSmoothingFactor = 0.9;
  // initial cost to change lanes to avoid frequent lane changes
  constexpr static int kInitialCostToChangeLane = 10000;
  // cost factor to penalize Time to Collision
  constexpr static int kCostFactorTTC = 2000;
  // cost factor to penalize Clear Distance (anterior distance without obstacles)
  constexpr static int kCostFactorClearDistance = 3000;
  // cost to penalize lane changes
  constexpr static int kCostChangeLane = 350;
  // cost for not being in the center lane
  constexpr static int kCostNotInTheCenterLane = 150;
  // cost to forbid critical situations like going outside or changing lane while forbidden
  constexpr static int kCostCritical = 1000;

public:
  PathPlanner();
  PathPlanner(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed);
  static void loadMap(std::string map_file);
  void setPreviousPath(const std::vector<double>& previous_path_x, const std::vector<double>& previous_path_y);
  void setSensorFusion(std::vector<ObstacleInfo>& sensor_fusion);
  void processSensorFusion();
  void decideTargetLane();
  void planPath();
  void planSpeed();
  Point generatePath(unsigned time_step);

private:
  Point transformWorld2Vehicle(double x_world, double y_world);
  Point transformVehicle2World(double x_vehicle, double y_vehicle);
  RelativeVelocity transformVelocity2RelativeVelocity(double vx_world, double vy_world);
};
