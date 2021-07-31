#pragma once
#include "common_structs.h"

namespace carND {

struct BehaviourPlanner {
  BehaviourPlanner(std::shared_ptr<Car> const &car);
  Behaviour getBehaviours(Predictions const & prediction);

private:
  std::shared_ptr<Car> m_car;
};
}  // namespace carND
