#include "path_planner.hpp"

#include <fstream>
#include <sstream>
#include <utility>
#include <algorithm>
#include <cmath>
#include <limits>

#include <iostream>
#include <iomanip>

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

inline double mph2ms(double mph) { return mph*0.44704; }
inline double ms2mph(double ms) { return ms*2.23694; }

inline int d2lane(double d) { return round((d+2.)/4.); }
inline int lane2d(int lane) { return 4 * lane - 2; }

double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

double distance(const Point& p1, const Point& p2)
{
  return distance(p1.x,p1.y,p2.x,p2.y);
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen){
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4) {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
Point getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) )) {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

vector<double> PathPlanner::waypoints_x, PathPlanner::waypoints_y, PathPlanner::waypoints_s;
double PathPlanner::target_speed;
PathPlanner::State PathPlanner::state;
int PathPlanner::target_lane;
int PathPlanner::cost_keep_lane, PathPlanner::cost_change_lane_to_right, PathPlanner::cost_change_lane_to_left;
int PathPlanner::keep_lane_count;

const double PathPlanner::max_speed;
const double PathPlanner::kSmoothingFactor;

PathPlanner::PathPlanner()
{
  PathPlanner::target_speed = max_speed;
  PathPlanner::state = PathPlanner::INIT;
}

PathPlanner::PathPlanner(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed)
{
  x = car_x; y = car_y;
  s = car_s; d = car_d;
  current_lane = d2lane(d);
  yaw = deg2rad(car_yaw); speed = mph2ms(car_speed);
  cos_yaw = cos(yaw); sin_yaw = sin(yaw);
  do_not_go_right = (current_lane>=3);
  do_not_go_left = (current_lane<=1);
  if (state == KEEP_LANE) {
    keep_lane_count = min(keep_lane_count+1,INT32_MAX-1);
    cost_keep_lane *= kSmoothingFactor;
    cost_change_lane_to_right *= kSmoothingFactor;
    cost_change_lane_to_left *= kSmoothingFactor;
  } else {
    keep_lane_count = 0;
    PathPlanner::cost_keep_lane = 0;
    PathPlanner::cost_change_lane_to_right = PathPlanner::kInitialCostToChangeLane;
    PathPlanner::cost_change_lane_to_left = PathPlanner::kInitialCostToChangeLane;
  }
}

void PathPlanner::loadMap(std::string map_file)
{
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  ifstream in_map_(map_file.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x, y;
    float s, d_x, d_y;
    iss >> x >> y >> s >> d_x >> d_y;
    PathPlanner::waypoints_x.push_back(x);
    PathPlanner::waypoints_y.push_back(y);
    PathPlanner::waypoints_s.push_back(s);
  }
}

void PathPlanner::setPreviousPath(const vector<double>& previous_path_x, const vector<double>& previous_path_y)
{
  for (int ii=0; ii<4 && ii<previous_path_x.size(); ++ii) {
    path_x.push_back(previous_path_x[ii]);
    path_y.push_back(previous_path_y[ii]);
  }
}

void PathPlanner::setSensorFusion(vector<vector<double>>& sensor_fusion)
{
  this->sensor_fusion = sensor_fusion;
}

void PathPlanner::processSensorFusion()
{
  // set the maximum value of each variables to calculate cost in this cycle
  min_time_to_collision = 100;
  min_distance_to_obstacle = 1000;
  ttc_left_lane = ttc_current_lane = ttc_right_lane = 99;
  clear_distance_left_lane = clear_distance_current_lane = clear_distance_right_lane = 99;

  // update variables with the information of sensor_fusion
  for (ObstacleInfo& obstacle : sensor_fusion) {
    int id = obstacle[0];
    double o_x = obstacle[1];
    double o_y = obstacle[2];
    double o_vx = obstacle[3];
    double o_vy = obstacle[4];
    double o_s = obstacle[5];
    double o_d = obstacle[6];

    Point point_obstacle = transformWorld2Vehicle(o_x,o_y);

    // Check if there are vehicles around left/right of the vehicle and forbit/stop lane change if so.
    if (point_obstacle.x < 10 && point_obstacle.x > -10) {
      if (point_obstacle.y > 1 && point_obstacle.y < 5) {
	do_not_go_left = true;
      } else if (point_obstacle.y < -1 && point_obstacle.y > -5) {
	do_not_go_right = true;
      }
    }

    // Transform the velocity of other vehicles to my vehicle coordinates
    RelativeVelocity velocity_obstacle = transformVelocity2RelativeVelocity(o_vx,o_vy);
    // Calculate time_to_collision which indicates the possibility of collision to this obstacle
    double time_to_collision = point_obstacle.x / -velocity_obstacle.x;
    // Does not care obstacles of negative TTC or posterior obstacles
    if (time_to_collision <= 0 || point_obstacle.x < 0)
      continue;
    // update variables for speed generator from the obstacles of in front of this vehicle
    if (abs(point_obstacle.y)<2 && point_obstacle.x > 0) {
      min_distance_to_obstacle = min(min_distance_to_obstacle, point_obstacle.x);
      min_time_to_collision = min(min_time_to_collision, time_to_collision);
    }

    // update variables for lane change planner depending on left/current/right lane each obstacle exists
    int obstacle_lane = d2lane(o_d);
    if (state == KEEP_LANE) {
      if (obstacle_lane==target_lane) {
	ttc_current_lane = min(ttc_current_lane, time_to_collision);
	clear_distance_current_lane = min(clear_distance_current_lane, point_obstacle.x);
      } else if (obstacle_lane==target_lane-1) {
	ttc_left_lane = min(ttc_left_lane, time_to_collision);
	clear_distance_left_lane = min(clear_distance_left_lane, point_obstacle.x);
      } else if (obstacle_lane==target_lane+1) {
	ttc_right_lane = min(ttc_right_lane, time_to_collision);
	clear_distance_right_lane = min(clear_distance_right_lane, point_obstacle.x);
      }
    }
  }
}

void PathPlanner::decideTargetLane()
{
  switch (state) {
  case INIT:
    target_lane = d2lane(d);
    state = KEEP_LANE;
    break;
  case KEEP_LANE:
    {
      auto speed_margin = max_speed-ms2mph(speed);

      // Penalizing lane changing because it should not be executed without needs
      cost_change_lane_to_right += kCostChangeLane;
      cost_change_lane_to_left += kCostChangeLane;

      // Penalize if there are slow vehicles ahead
      cost_keep_lane += kCostFactorTTC/ttc_current_lane;
      cost_change_lane_to_right += kCostFactorTTC/ttc_right_lane;
      cost_change_lane_to_left += kCostFactorTTC/ttc_left_lane;

      // Penalize if there are so much space ahead
      cost_keep_lane += kCostFactorClearDistance/clear_distance_current_lane;
      cost_change_lane_to_right += kCostFactorClearDistance/clear_distance_right_lane;
      cost_change_lane_to_left += kCostFactorClearDistance/clear_distance_left_lane;

      // Penalizing going slowly in current lane
      cost_keep_lane += speed_margin * 35;
      // Penalizing lane changes while speed is slow to avoid collision while changing lane
      cost_change_lane_to_right += speed_margin*speed_margin;
      cost_change_lane_to_left += speed_margin*speed_margin;

      // Penalize for not being in the center lane because center lane has more choices of behavior
      cost_keep_lane += (target_lane!=2 ? kCostNotInTheCenterLane : 0);

      // Penalize lane change if do_not_go_left/right signals are emitted
      cost_change_lane_to_right += (do_not_go_right ? kCostCritical:0);
      cost_change_lane_to_left += (do_not_go_left ? kCostCritical:0);

      // Penalize changing lane to Out of Road
      if (current_lane==1)
	cost_change_lane_to_left += kCostCritical;
      else if (current_lane==1)
	cost_change_lane_to_right += kCostCritical;

      // Forbid lane change if the car is not keeping lane enough to avoid staying in the middle of lanes
      if (keep_lane_count < kMinimumKeepLaneCount)
	do_not_go_left = do_not_go_right = true;

      cerr << "COST to go left:" << setw(7) << cost_change_lane_to_left << " keep:" << setw(7) << cost_keep_lane << " go right:" << setw(7) << cost_change_lane_to_right << endl;

      double minimum_cost = min(cost_keep_lane, min(cost_change_lane_to_right, cost_change_lane_to_left));

      // Change lane if it is not forbidden and changing lane is better than keeping lane
      if (!do_not_go_left && minimum_cost == cost_change_lane_to_left) {
	state = CHANGE_LANE_LEFT;
	--target_lane;
	cerr << "Change lane LEFT to lane=" << target_lane << endl;
      } else if (!do_not_go_right && minimum_cost == cost_change_lane_to_right) {
	state = CHANGE_LANE_RIGHT;
	++target_lane;
	cerr << "Change lane RIGHT to lane=" << target_lane << endl;
      }
    }
    break;
  case CHANGE_LANE_RIGHT:
    if (target_lane == d2lane(d)) {
      // Terminating lane change state
      state = KEEP_LANE;
    } else if (do_not_go_right) {
      // Stopping lane changing behavior to avoid collision
      state = KEEP_LANE;
      --target_lane;
    }
    break;
  case CHANGE_LANE_LEFT:
    if (target_lane == d2lane(d)) {
      // Terminating lane change state
      state = KEEP_LANE;
    } else if (do_not_go_left) {
      // Stopping lane changing behavior to avoid collision
      state = KEEP_LANE;
      ++target_lane;
    }
    break;
  }
}

void PathPlanner::initializePath()
{
  // Put some key points ahead on the target lane
  vector<Point> key_points_world;
  Point point_far1 = getXY(s + 40, lane2d(target_lane), waypoints_s, waypoints_x, waypoints_y);
  Point point_far2 = getXY(s + 50, lane2d(target_lane), waypoints_s, waypoints_x, waypoints_y);
  // For the first cycle, key points starts from the current vehicle position
  if (path_x.size() < 2) {
    double easing_length = 0.02 * max(1.0,speed);
    Point point_now = {x, y};
    Point point_prev = {x - easing_length * cos_yaw, y - easing_length * sin_yaw};

    key_points_world = {point_prev, point_now, point_far1, point_far2};
    start_speed = speed;

    // Later, key points continues from the points from last cycle
  } else {
    Point point_last = {*(path_x.end()-1),*(path_y.end()-1)};
    Point point_last2 = {*(path_x.end()-2),*(path_y.end()-2)};

    key_points_world = {point_last2, point_last, point_far1, point_far2};
    start_speed = distance(point_last,point_last2) * 50;
  }

  // Convert coordinates from world to vehicle so as to make x always ascending
  vector<double> key_points_vehicle_x, key_points_vehicle_y;
  for (Point& key_point : key_points_world) {
    Point key_point_vehicle = transformWorld2Vehicle(key_point.x, key_point.y);
    key_points_vehicle_x.push_back(key_point_vehicle.x);
    key_points_vehicle_y.push_back(key_point_vehicle.y);
  }
  path_point_x = key_points_vehicle_x[1];

  spline_path.set_points(key_points_vehicle_x, key_points_vehicle_y);
}

void PathPlanner::generateSpeed()
{
  // Decrease/Increase speed depenging on TTC
  if (min_time_to_collision < 0)
    target_speed -= 0.2;
  else if (min_time_to_collision < 5)
    target_speed -= 0.4;
  else if (min_time_to_collision < 10)
    target_speed -= 0.3;
  else if (min_time_to_collision < 15)
    target_speed -= 0.2;
  else if (min_time_to_collision < 20)
    target_speed -= 0.1;
  else if (min_time_to_collision < 45)
    ;
  else if (min_time_to_collision < 60)
    target_speed += 0.2;
  else
    target_speed += 0.4;

  // Decrease/Increase speed depenging on distance ahead without obstacles
  if (min_distance_to_obstacle < 5)
    target_speed -= 0.8;
  else if (min_distance_to_obstacle < 10)
    target_speed -= 0.6;
  else if (min_distance_to_obstacle < 15)
    target_speed -= 0.4;
  else if (min_distance_to_obstacle < 20)
    target_speed -= 0.2;
  else
    ;

  // Clamp target speed from 1.0 to max_speed
  target_speed = max(1.0, min(max_speed, target_speed));

  // Make a spline which aims to achieve target speed in 125 frames
  vector<double> key_points_t, key_points_speed;
  key_points_t.push_back(  0); key_points_speed.push_back(start_speed);
  key_points_t.push_back(125); key_points_speed.push_back(min(start_speed+2,mph2ms(target_speed)));
  key_points_t.push_back(150); key_points_speed.push_back(min(start_speed+2,mph2ms(target_speed)));
  spline_speed.set_points(key_points_t, key_points_speed);
}

Point PathPlanner::generatePath(unsigned time_step)
{
  unsigned num_previous_path = path_x.size();
  if (time_step < num_previous_path)
    return {path_x[time_step],path_y[time_step]};

  time_step = time_step - num_previous_path + 1;
  double distance_per_cycle = spline_speed(time_step) * 0.02;
  path_point_x += distance_per_cycle;
  double path_point_y = spline_path(path_point_x);
  // converting to world coordinates
  return transformVehicle2World(path_point_x,path_point_y);
}

Point PathPlanner::transformWorld2Vehicle(double x_world, double y_world)
{
  double x_diff = x_world - x;
  double y_diff = y_world - y;

  return {x_diff * cos_yaw + y_diff * sin_yaw,
          x_diff *-sin_yaw + y_diff * cos_yaw};
}

Point PathPlanner::transformVehicle2World(double x_vehicle, double y_vehicle)
{
  return {x + x_vehicle * cos_yaw - y_vehicle * sin_yaw,
          y + x_vehicle * sin_yaw + y_vehicle * cos_yaw};
}

RelativeVelocity PathPlanner::transformVelocity2RelativeVelocity(double vx_world, double vy_world)
{
  return {vx_world * cos_yaw + vy_world * sin_yaw - speed,
          vx_world *-sin_yaw + vy_world * cos_yaw};
}
