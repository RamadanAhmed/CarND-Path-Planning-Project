#include "trajectory_generator.h"
#include "helpers.h"
namespace carND {
Points TrajectoryGenerator::generate_trajectory(PreviousPath const &prev_path, const Behaviour behaviour) {
  Points new_path;
  auto prev_path_size = prev_path.points.x.size();
  
  m_car->lane = behaviour.lane;
  m_car->ref_velocity = behaviour.velocity;
  
  // Create Way Points
  Points way_points;
  double ref_x = m_car->x;
  double ref_y = m_car->y;
  double ref_yaw = deg2rad(m_car->yaw);

  if(prev_path_size < 2) {
    // if previous path is almost done (i.e empty) use the car as starting point
    // use two points to make a path tangent to the car
    double ref_x_prev = m_car->x - cos(m_car->yaw);
    double ref_y_prev = m_car->y - sin(m_car->yaw);

    way_points.x.push_back(ref_x_prev);
    way_points.y.push_back(ref_y_prev);
  } else {
    // if previous path is not done (i.e contain data) use the prev_path as starting point
    ref_x = prev_path.points.x[prev_path_size - 1];
    ref_y = prev_path.points.y[prev_path_size - 1];

    double ref_x_prev = prev_path.points.x[prev_path_size - 2];
    double ref_y_prev = prev_path.points.y[prev_path_size - 2];

    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    way_points.x.push_back(ref_x_prev);
    way_points.y.push_back(ref_y_prev);
  }
  way_points.x.push_back(ref_x);
  way_points.y.push_back(ref_y);
  // Add 3 spaced by 30m points In frenet coordinates
  vector<double> next_wp0 = getXY(m_car->s + 30, (2 + 4 * m_car->lane), m_map_waypoints_s,
                                  m_map_waypoints_x, m_map_waypoints_y);
  vector<double> next_wp1 = getXY(m_car->s + 60, (2 + 4 * m_car->lane), m_map_waypoints_s,
                                  m_map_waypoints_x, m_map_waypoints_y);
  vector<double> next_wp2 = getXY(m_car->s + 90, (2 + 4 * m_car->lane), m_map_waypoints_s,
                                  m_map_waypoints_x, m_map_waypoints_y);
  way_points.x.push_back(next_wp0[0]);
  way_points.x.push_back(next_wp1[0]);
  way_points.x.push_back(next_wp2[0]);

  way_points.y.push_back(next_wp0[1]);
  way_points.y.push_back(next_wp1[1]);
  way_points.y.push_back(next_wp2[1]);

  way_points = transform_to_car_coords(way_points, ref_x, ref_y, ref_yaw);

  m_spline.set_points(way_points.x, way_points.y);
  // add all remaining prev_path -> help in transition
  for(auto i = 0; i < prev_path_size; ++i) {
    new_path.x.push_back(prev_path.points.x[i]);
    new_path.y.push_back(prev_path.points.y[i]);
  }

  double target_x = 30.0;
  double target_y = m_spline(target_x);
  double target_distance = distance(target_x, target_x, target_y, target_y);

  double x_add_on = 0;
  
  
  double N = (target_distance / (0.02 * m_car->ref_velocity / 2.24));  
  for(int i = 1; i < 50 - prev_path_size; ++i) {
    auto x = x_add_on + (target_x) / N;
    auto y = m_spline(x);

    x_add_on = x;
    
    auto new_points = transform_to_global_coords(x, y, ref_x, ref_y, ref_yaw);
    new_path.x.push_back(new_points[0]);
    new_path.y.push_back(new_points[1]);
  }
  return new_path;
}
}  // namespace carND