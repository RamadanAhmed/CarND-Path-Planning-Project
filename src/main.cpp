#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
// user-defined
#include "common_structs.h"
#include "trajectory_generator.h"
#include "behaviour_planner.h"
#include "predictor.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while(getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  std::shared_ptr<carND::Car> car = std::make_shared<carND::Car>();
  carND::TrajectoryGenerator traj_generator(car, map_waypoints_x,
                                            map_waypoints_y, map_waypoints_s);
  carND::Predictor predictor(car);
  carND::BehaviourPlanner planner(car);
  h.onMessage([&map_waypoints_dx, &map_waypoints_dy, &car, &predictor,
               &traj_generator, &planner](uWS::WebSocket<uWS::SERVER> ws, char *data,
                                size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if(length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if(s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if(event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          car->set(j[1]["x"], j[1]["y"], j[1]["s"], j[1]["d"], j[1]["yaw"], j[1]["speed"]);

          carND::PreviousPath previous_path{{j[1]["previous_path_x"], j[1]["previous_path_y"]},
                                            j[1]["end_path_s"],
                                            j[1]["end_path_d"]};
          auto prev_size = previous_path.points.x.size();
          if(prev_size > 0) {
            car->s = previous_path.end_s;
          }
          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          std::vector<carND::SensorData> sensor_data;
          for(auto const &sensor : sensor_fusion) {
            sensor_data.emplace_back(sensor[0], sensor[1], sensor[2], sensor[3],
                                     sensor[4], sensor[5], sensor[6]);
          }

          json msgJson;
          auto predictions = predictor.getPredictions(sensor_data, prev_size);
          auto behaviour = planner.getBehaviours(predictions);
          auto next_val = traj_generator.generate_trajectory(previous_path, behaviour);

          msgJson["next_x"] = next_val.x;
          msgJson["next_y"] = next_val.y;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  });  // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
#ifdef WSL_ENABLED
  auto url = "127.0.0.1";
  if(h.listen(url, port)) {
#else
  if(h.listen(port)) {
#endif
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}