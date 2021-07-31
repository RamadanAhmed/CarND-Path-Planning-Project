#pragma once
#include "common_structs.h"
#include <vector>
#include <memory>

namespace carND {

struct Predictor {
    Predictor(std::shared_ptr<Car> const& car) : m_car{car}{}
    Predictions getPredictions(std::vector<SensorData> const& cars_data, const std::size_t prev_size);
private:
std::shared_ptr<Car> m_car;
};
}  // namespace carND