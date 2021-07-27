#pragma once
#include <vector>
#include <memory>
#include "spline.h"
#include "common_structs.h"
#include "helpers.h"

namespace carND {
struct TrajectoryGenerator {
  TrajectoryGenerator(std::shared_ptr<Car> const &car, std::vector<double> const &map_waypoints_x,
                      std::vector<double> const &map_waypoints_y,
                      std::vector<double> const &map_waypoints_s) :
      m_car{car},
      m_map_waypoints_s(map_waypoints_s), m_map_waypoints_x(map_waypoints_x),
      m_map_waypoints_y(map_waypoints_y){};
  Points generate_trajectory(PreviousPath const &prev_path,
                             std::vector<SensorData> const &sensors_data);

private:
  Points transform_to_car_coords(Points const &old_points, const double ref_x,
                                 const double ref_y, const double ref_yaw);
  Points transform_to_global_coords(Points const &old_points, const double ref_x,
                                    const double ref_y, const double ref_yaw);
  std::vector<double> transform_to_global_coords(const double old_x, const double old_y,
                                                 const double ref_x, const double ref_y,
                                                 const double ref_yaw);

private:
  int m_currentLane = 1;
  double m_ref_vel = 49.5;
  tk::spline m_spline;
  std::shared_ptr<Car> m_car;
  std::vector<double> m_map_waypoints_s;
  std::vector<double> m_map_waypoints_x;
  std::vector<double> m_map_waypoints_y;
};
}  // namespace carND