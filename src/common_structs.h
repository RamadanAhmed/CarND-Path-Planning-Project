#pragma once
#include <vector>

namespace carND {
struct Car {
  void set(double pX, double pY, double pS, double pD, double pYaw, double pSpeed) {
    x = pX;
    y = pY;
    s = pS;
    d = pD;
    yaw = pYaw;
    speed = pSpeed;
  }
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
  double ref_velocity = 0.0;
  int lane = 1;
};

struct SensorData {
  SensorData(int pId, double pX, double pY, double pVx, double pVy, double pS, double pD) :
      id{pId}, x{pX}, y{pY}, vx{pVx}, vy{pVy}, s{pS}, d{pD} {}
  int id = 0;
  double x = 0.0;
  double y = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double s = 0.0;
  double d = 0.0;
};

struct Points {
  std::vector<double> x;
  std::vector<double> y;
};

struct PreviousPath {
  Points points;
  double end_s;
  double end_d;
};

struct Behaviour {
  int lane;
  double velocity;
};

struct Predictions{
    bool car_ahead = false;
    bool car_right = false;
    bool car_left = false;
};
}  // namespace carND
