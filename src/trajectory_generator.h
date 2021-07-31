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
  Points generate_trajectory(PreviousPath const &prev_path, const Behaviour behaviour);

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