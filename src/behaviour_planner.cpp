// STL
#include <memory>
// user-defined
#include "behaviour_planner.h"

namespace carND {
BehaviourPlanner::BehaviourPlanner(std::shared_ptr<Car> const &car) :
    m_car(car) {}

Behaviour BehaviourPlanner::getBehaviours(Predictions const &prediction) {
  Behaviour res_behave;
  double speed_diff = 0;
  constexpr double MAX_SPEED = 49.5;
  constexpr double MAX_ACC = .224;
  res_behave.lane = m_car->lane;
  if(prediction.car_ahead) {
    if(!prediction.car_left && m_car->lane > 0) {
      res_behave.lane = m_car->lane - 1;
    } else if(!prediction.car_right && m_car->lane != 2) {
      res_behave.lane = m_car->lane + 1;
    } else {
      speed_diff -= MAX_ACC;
    }
  } else {
    if(m_car->lane != 1) {
      if((m_car->lane == 0 && !prediction.car_right) ||
         (m_car->lane == 2 && !prediction.car_left)) {
        res_behave.lane = 1;
      }
    }
    if(m_car->ref_velocity < MAX_SPEED) {
      speed_diff += MAX_ACC;
    }
  }

  res_behave.velocity = m_car->ref_velocity + speed_diff;
  if(res_behave.velocity > MAX_SPEED) {
    res_behave.velocity = MAX_SPEED;
  } else if(res_behave.velocity < MAX_ACC) {
    res_behave.velocity = MAX_ACC;
  }

  return res_behave;
}
}  // namespace carND
