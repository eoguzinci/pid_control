#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <limits.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

// Equivalent of Make_Robot()
void reset_simulator(uWS::WebSocket<uWS::SERVER>& ws)
{
  std::string msg("42[\"reset\", {}]");
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  std::vector<double> p = {0.1, 0.001, 10.0};
  std::vector<double> dp = {0.01, 0.0001,1.0};
  pid.Init(p[0], p[1], p[2]);

  double sum_dp = 0.0;
  for(unsigned int i=0; i < 3; i++){
    sum_dp += dp[i];
  }
  int param_index = 2;

  bool twiddle = true;
  unsigned int iter = 0;
  double best_error = std::numeric_limits<double>::max();
  double current_error = 0;
  double total_distance = 0.0 ;
  const unsigned int max_iter = 200;

  h.onMessage([&pid, &p, &dp, &twiddle, &iter, &best_error, &current_error, &sum_dp, &total_distance, &max_iter, &param_index](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          double throttle;

          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          std::cout << "sum_dp: " << sum_dp << std::endl;
          std::cout << "param_index: " << param_index << std::endl;
          // Twiddle algorithm
          if(twiddle){
            if(iter<max_iter && sum_dp>0.0001){
              if(iter > 0){
                p[param_index] += dp[param_index];
                pid.Init(p[0], p[1], p[2]);
                reset_simulator(ws);
                pid.Run(cte, steer_value, throttle, speed, current_error);
                std::cout << "++dp: curr_error=" << current_error << " best_error="<< best_error << std::endl;
                if(current_error < best_error){
                  best_error = current_error;
                  dp[param_index] *= 1.1;
                } 
                else{
                  p[param_index] -= 2*dp[param_index];
                  pid.Init(p[0], p[1], p[2]);
                  reset_simulator(ws);
                  pid.Run(cte, steer_value, throttle, speed, current_error);
                  std::cout << "--dp: curr_error=" << current_error << " best_error="<< best_error << std::endl;
                  if(current_error < best_error){
                    best_error = current_error;
                    dp[param_index] *= 1.1;
                  }
                  else{
                    std::cout << "Neutral -> dp *= 0.9" << std::endl;
                    p[param_index] += dp[param_index];
                    dp[param_index] *= 0.9;
                  }
                }
              }else if(iter == 0){
                pid.Run(cte, steer_value, throttle, speed, current_error);
                best_error = current_error;
              }            
              param_index = (param_index+1)% p.size();
              sum_dp = 0.0;
              for(unsigned int i=0; i<dp.size(); i++){
                sum_dp += dp[i];
              }
              iter ++;
            }else{
              pid.Run(cte, steer_value, throttle, speed, current_error);
            }
          }else{
            pid.Run(cte, steer_value, throttle, speed, current_error);
          }

          // DEBUG
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          if (iter > max_iter-1){
            std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " speed: " << speed << std::endl;
            std::cout << "Kp = " << pid.Kp << ", Ki = " << pid.Ki << ", Kd = " << pid.Kd <<std::endl;
          } else {
            std::cout << "iter: " << iter << ", best_error = " << best_error << ", sum_dp = "<< sum_dp << std::endl;
            std::cout << "Kp = " << pid.Kp << ", Ki = " << pid.Ki << ", Kd = " << pid.Kd <<std::endl;
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle; // variable speed
          // msgJson["throttle"] = 0.3; // fixed speed
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
