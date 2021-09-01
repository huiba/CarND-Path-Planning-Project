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
#include "state.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

double get_certer_d_from_lane(int lane)
{
  return (double)2 + 4 * lane; 
}

int get_lane_from_d(double d)
{
  return floor(d / 4);
}

double mph2mps(double mph)
{
  return mph * 0.44704; 
}

double mps2mph(double mps)
{
  return mps * 2.23694;
}

double min_cost_index(double a, double b, double c)
{
  double m = a;
  int idx = 0;
  if (b < m) {
    m = b;
    idx = 1;
  }
  if (c < m) {
    m = c;
    idx = 2;
  }
  return idx;
}
/* 
vector<int> get_next(int state) 
{

}

double cost(vector<double> predicted_state, vector<vector<double>> predictions)
{
}
*/

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
  while (getline(in_map_, line)) {
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

  int lane = 1;
  double ref_vel = 0.0; // mph
  // state: 0: keep lane, 1: change left lane, 2: change right lane
  int state = 0;

  const double SPEED_LIMIT = 49.5; //mph



  h.onMessage([&lane, &ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          double pos_x;
          double pos_y;
          double angle;
          int prev_size = previous_path_x.size();

          // sensor fusion
          if (prev_size > 0) {
            car_s = end_path_s;
          }
          int our_lane = get_lane_from_d(car_d);
          // car info: (s, d, vx, vy, speed, dist)
          double max_dist = std::numeric_limits<double>::max();
          vector<double> front_car = {-1, -1, -1, -1, -1, max_dist}; 
          vector<double> left_front_car = {-1, -1, -1, -1, -1, max_dist};
          vector<double> left_back_car = {-1, -1, -1, -1, -1, max_dist};
          vector<double> right_front_car = {-1, -1, -1, -1, -1, max_dist};
          vector<double> right_back_car = {-1, -1, -1, -1, -1, max_dist};

          // find most relevant cars
          double min_front_dist = std::numeric_limits<double>::max();
          double min_left_front_dist = std::numeric_limits<double>::max();
          double min_left_back_dist = std::numeric_limits<double>::max();
          double min_right_front_dist = std::numeric_limits<double>::max();
          double min_right_back_dist = std::numeric_limits<double>::max();

          for (int i = 0; i < sensor_fusion.size(); ++i) {
            double f_s = sensor_fusion[i][5];
            double f_d = sensor_fusion[i][6];
            double f_vx = sensor_fusion[i][3];
            double f_vy = sensor_fusion[i][4];
            double f_speed = sqrt(f_vx * f_vx + f_vy * f_vy);
            f_s += (double) prev_size * 0.02 * f_speed;
            int f_lane = get_lane_from_d(f_d);

            double dist = car_s - f_s;
            double abs_dist = std::abs(dist);
            //1. same lane
            if (our_lane == f_lane && dist <= 0) {
              if (abs_dist < min_front_dist) {
                min_front_dist = abs_dist;
                front_car[0] = f_s;
                front_car[1] = f_d;
                front_car[2] = f_vx;
                front_car[3] = f_vy;
                front_car[4] = f_speed;
                front_car[5] = abs_dist;
              }
            } 
            //2. left lane
            else if (our_lane - f_lane == 1) {
              if (dist <= 0) {
                // front
                if (abs_dist < min_left_front_dist) {
                  min_left_front_dist = abs_dist;
                  left_front_car[0] = f_s;
                  left_front_car[1] = f_d;
                  left_front_car[2] = f_vx;
                  left_front_car[3] = f_vy;
                  left_front_car[4] = f_speed;
                  left_front_car[5] = abs_dist;
                }
              } else {
                // back
                if (abs_dist < min_left_back_dist) {
                  min_left_back_dist = abs_dist;
                  left_back_car[0] = f_s;
                  left_back_car[1] = f_d;
                  left_back_car[2] = f_vx;
                  left_back_car[3] = f_vy;
                  left_back_car[4] = f_speed;
                  left_back_car[5] = abs_dist;
                }
              }
            } 
            //3. right lane
            else if (our_lane - f_lane == -1) {
              if (dist <= 0) {
                // front
                if (abs_dist < min_right_front_dist) {
                  min_right_front_dist = abs_dist;
                  right_front_car[0] = f_s;
                  right_front_car[1] = f_d;
                  right_front_car[2] = f_vx;
                  right_front_car[3] = f_vy;
                  right_front_car[4] = f_speed;
                  right_front_car[5] = abs_dist;
                }
              } else {
                // back
                if (abs_dist < min_right_back_dist) {
                  min_right_back_dist = abs_dist;
                  right_back_car[0] = f_s;
                  right_back_car[1] = f_d;
                  right_back_car[2] = f_vx;
                  right_back_car[3] = f_vy;
                  right_back_car[4] = f_speed;
                  right_back_car[5] = abs_dist;
                }
              } 
            }
            else { 
              continue; 
            }
          }// for sensor fusion

          double max_cost = std::numeric_limits<double>::max();
          // cost for keeping lane
          double cost_keeping_lane = 0;
          const double DIST_LIMIT_FRONT = 20;
          const double DIST_LIMIT_SIDE_FRONT = 20;
          const double DIST_LIMIT_SIDE_BACK = 20;
          bool close = false;
          if (front_car[5] < DIST_LIMIT_FRONT) {
            close = true;
            cost_keeping_lane = 100;
          }
          // cost for changing to left lane
          double cost_change_left_lane=0;
          if (our_lane > 0) {
            if (left_front_car[5] < DIST_LIMIT_SIDE_FRONT || left_back_car[5] < DIST_LIMIT_SIDE_BACK) {
              cost_change_left_lane = 200;
            } 
          } else {
            cost_change_left_lane = max_cost;
          }
          // cost for changing to right lane
          double cost_change_right_lane = 0;
          if (our_lane < 2) {
            if (right_front_car[5] < DIST_LIMIT_SIDE_FRONT || right_back_car[5] < DIST_LIMIT_SIDE_BACK) {
              cost_change_right_lane = 200;
            } 
          } else {
            cost_change_right_lane = max_cost;
          }
          // 0 -> keep lane, 1 -> change left lane, 2 -> change right lane
          int decision = min_cost_index(cost_keeping_lane, cost_change_left_lane, cost_change_right_lane);
          double acc = 0.4;
          if (decision == 1) {
            lane = our_lane - 1;
          }
          if (decision == 2) {
            lane = our_lane + 1;
          }
          if (decision == 0 && close) {
            ref_vel -= acc;
          }
          else if (ref_vel < 49.5) {
            ref_vel += acc;
          }

          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          if (prev_size < 2) {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);

          }
          else {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          vector<double> next_wp0 = getXY(car_s + 30, get_certer_d_from_lane(lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, get_certer_d_from_lane(lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, get_certer_d_from_lane(lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
        
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (int i = 0; i < ptsx.size(); ++i) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
            ptsy[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
          }

          tk::spline s;
          s.set_points(ptsx, ptsy);

          for (int i = 0; i < prev_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);
          double x_add_on = 0;


          for (int i = 0; i < 50 - prev_size; ++i) {
            double N = target_dist / (0.02 * ref_vel / 2.24);
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);
            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }



          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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