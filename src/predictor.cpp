#include "predictor.h"
#include <cmath>

namespace carND {
Predictions Predictor::getPredictions(std::vector<SensorData> const &cars_data, const std::size_t prev_size) {
  Predictions result_pred;
  for(auto const &car : cars_data) {
    auto current_car_lane = (car.d > 0 && car.d < 4)    ? 0
                            : (car.d > 4 && car.d < 8)  ? 1
                            : (car.d > 8 && car.d < 12) ? 2
                                                        : -1;
    if(current_car_lane >= 0) {
        auto check_speed = std::sqrt(car.vx * car.vx + car.vy * car.vy);
        auto check_s = car.s + double(prev_size * 0.02 * check_speed);
        
        if(current_car_lane == m_car->lane) {
            result_pred.car_ahead |= check_s > m_car->s && check_s - m_car->s < 30;
        }
        else if((current_car_lane - m_car->lane) == -1) {
            result_pred.car_left |= m_car->s - 30 < check_s && m_car->s + 30 > check_s;
        }
        else if ((current_car_lane - m_car->lane) == 1) {
            result_pred.car_right |= m_car->s - 30 < check_s && m_car->s + 30 > check_s;
        }
    }
  }
  return result_pred;
}
}  // namespace carND
