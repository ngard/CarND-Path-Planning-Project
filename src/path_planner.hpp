#pragma once

#include <vector>
#include <string>
#include "spline.h"

typedef struct Point {
  double x;
  double y;
} Point;

class PathPlanner {
private:
  double x, y, s, d, yaw, speed;
  double cos_yaw, sin_yaw;
  tk::spline spline_path, spline_speed;
  std::vector<double> path_x, path_y;
  double start_speed;
  double path_point_x;

public:
  static std::vector<double> waypoints_x, waypoints_y, waypoints_s;

  PathPlanner();
  PathPlanner(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed);
  static void loadMap(std::string map_file);
  void setPreviousPath(const std::vector<double>& previous_path_x, const std::vector<double>& previous_path_y);
  void initializePath();
  void initializeSpeed(double target_speed);
  Point generatePath(unsigned time_step);

private:
  Point transformWorld2Vehicle(double x_world, double y_world);
  Point transformVehicle2World(double x_vehicle, double y_vehicle);
};
