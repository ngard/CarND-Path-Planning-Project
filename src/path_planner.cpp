#include "path_planner.hpp"

#include <fstream>
#include <sstream>
#include <utility>
#include <algorithm>
#include <cmath>

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

inline double mph2ms(double mph) { return mph*0.44704; }

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

PathPlanner::PathPlanner()
{}

PathPlanner::PathPlanner(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed)
{
  x = car_x; y = car_y;
  s = car_s; d = car_d;
  yaw = deg2rad(car_yaw); speed = car_speed;
  cos_yaw = cos(yaw); sin_yaw = sin(yaw);
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

void PathPlanner::initializePath()
{
  // Make some key points which roughly determines vehicle motion
  vector<Point> key_points_world;
  Point point_far1 = getXY(s + 30, 6.0, waypoints_s, waypoints_x, waypoints_y);
  Point point_far2 = getXY(s + 50, 6.0, waypoints_s, waypoints_x, waypoints_y);
  // For the first cycle, key points starts from the current vehicle position
  if (path_x.size() < 2) {
    double easing_length = 0.02 * mph2ms(max(2.0,speed));
    Point point_now = {x, y};
    Point point_prev = {x - easing_length * cos_yaw, y - easing_length * sin_yaw};

    key_points_world = {point_prev, point_now, point_far1, point_far2};
    start_speed = mph2ms(speed);

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

void PathPlanner::initializeSpeed(double target_speed)
{
  vector<double> key_points_t, key_points_speed;
  key_points_t.push_back( 0); key_points_speed.push_back(start_speed);
  key_points_t.push_back(25); key_points_speed.push_back(min(start_speed+2,mph2ms(target_speed)));
  key_points_t.push_back(30); key_points_speed.push_back(min(start_speed+2,mph2ms(target_speed)));
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
