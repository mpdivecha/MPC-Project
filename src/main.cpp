#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

const std::string red("\033[0;31m");
const std::string green("\033[1;32m");
const std::string yellow("\033[1;33m");
const std::string cyan("\033[0;36m");
const std::string magenta("\033[0;35m");
const std::string reset("\033[0m");

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  vector<double> cost_multipliers;
  cost_multipliers.push_back(6000);         // CTE cost
  cost_multipliers.push_back(6000);         // Epsi cost 
  cost_multipliers.push_back(1);            // v cost
  cost_multipliers.push_back(10);           // delta actuator use cost
  cost_multipliers.push_back(10);           // a actuator use cost
  cost_multipliers.push_back(200);          // delta actuator gap cost
  cost_multipliers.push_back(10);           // a actuator gap cost
  MPC mpc(cost_multipliers);

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << "sdata: " << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];

          for (int i = 0; i < ptsx.size(); i++)
          {
              // shift car reference by 90 degrees
              double shift_x = ptsx[i] - px;
              double shift_y = ptsy[i] - py;
              // Coordinate transform to vehicle coordinates
              ptsx[i] = (shift_x*cos(-psi) - shift_y*sin(-psi));
              ptsy[i] = (shift_x*sin(-psi) + shift_y*cos(-psi));
          }

          Eigen::Map<Eigen::VectorXd> ptsx_map(&ptsx[0], 6);
          Eigen::Map<Eigen::VectorXd> ptsy_map(&ptsy[0], 6);

          auto coeffs = polyfit(ptsx_map, ptsy_map, 3);

          double cte = polyeval(coeffs, 0);
          double epsi = -atan(coeffs[1]);

          // Here we generate the current state predictions
          double latency = 0.1;
          double Lf = 2.67;

          double state_x = 0 + v*cos(0)*latency;
          double state_y = 0;
          double state_psi = 0 - v*steer_value/Lf*latency;
          double state_v = v + throttle_value*latency; 
          double state_cte = cte + v*sin(epsi)*latency;
          double state_epsi = epsi - v*steer_value/Lf*latency;

          Eigen::VectorXd state(6);
          state << state_x, state_y, state_psi, state_v, state_cte, state_epsi;

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          auto vars = mpc.Solve(state, coeffs);

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          steer_value = vars[0]/(deg2rad(25)*Lf);
          throttle_value = vars[1];

          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          if (abs(state_cte) > 1) cout << red;
          cout << "steer: " << steer_value << " throttle: " << throttle_value;
          cout << " cte: " << state_cte << " epsi: " << epsi;
          cout << " curvature: " << pow(coeffs[3]*100*state_v,2 ) << endl << reset;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          for (int i = 2; i < vars.size(); i++)
          {
              if (i%2 == 0)
              {
                  mpc_x_vals.push_back(vars[i]);
              }
              else
              {
                  mpc_y_vals.push_back(vars[i]);
              }
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          double poly_inc = 2.5;
          int num_points = 25;
          for (int i = 1; i < num_points; i++)
          {
              next_x_vals.push_back(poly_inc * i);
              next_y_vals.push_back(polyeval(coeffs, poly_inc * i));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << "msg: " << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
