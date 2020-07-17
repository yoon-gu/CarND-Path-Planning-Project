#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using namespace std;

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
  double ref_vel = 0; // mph
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane,&ref_vel, &max_s]
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

          // constant values
          const int left_most_lane = 0;
          const int right_most_lane = 2;
          const double max_speed = 49.5;
          const double speed_delta = .224;
          const double ahead_dist_limit = 30;
          const double behind_dist_limit = 30;

          int prev_size = previous_path_x.size();// Provided previous path point size.
          if (prev_size > 0)
            car_s = end_path_s;

          map<int, double> speed_map; // key: id of a car
          map<int, double> s_pos_map;

          vector<int> ahead_center_ids;
          vector<int> ahead_left_ids;
          vector<int> ahead_right_ids;
          vector<int> behind_center_ids;
          vector<int> behind_left_ids;
          vector<int> behind_right_ids;

          // Predict neighbor cars 's' pos and speed after previous path points * 20 ms
          for ( int i = 0; i < sensor_fusion.size(); i++ )
          {
            int id = sensor_fusion[i][0];
            double x = sensor_fusion[i][1];
            double y = sensor_fusion[i][2];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double sensor_s = sensor_fusion[i][5]; // Frenet coordinate for the car
            double d = sensor_fusion[i][6]; // Frenet coordinate for the car

            int sensor_lane = -1;
            sensor_lane = (int)(d / 4); // each lane width = 4 m

            double speed_check = sqrt(vx * vx + vy * vy);
            double dt = (double) prev_size * 0.02; // 20 ms
            double check_car_s = sensor_s;

            // Check exceptional case at the end of highway
            if ( (car_s + ahead_dist_limit > max_s) && (check_car_s < ahead_dist_limit))
                check_car_s += max_s;
            // Check exceptional case at the end of highway
            if ( (car_s - behind_dist_limit < 0) && (check_car_s > max_s - behind_dist_limit))
                check_car_s -= max_s;

            check_car_s += (dt * speed_check);

            double theta = atan2(vy, vx);
            vector<double> predict_sd = getFrenet(x + vx * dt, y + vy * dt, theta, map_waypoints_x, map_waypoints_y);

            int sensor_lane2 = (int)(predict_sd[1] / 4);
            // To avoid exceptional case
            if (d == 0.0 && predict_sd[1] > 0)
            {
                d = predict_sd[1];
                sensor_lane = sensor_lane2;
            }

            speed_map[id] = speed_check * 2.24; // mph from mps
            s_pos_map[id] = check_car_s;

            // Check a car by {ahead, behind} for {left, center, right}
            auto check_ahead_behind = [&](vector<int>& ahead_ids, vector<int>& behind_ids)
                {
                  double distance = check_car_s - car_s;
                  if (distance > 0 )
                      ahead_ids.push_back(id);
                  else
                      behind_ids.push_back(id);
                };

            if ( sensor_lane == lane )
            {
                check_ahead_behind(ahead_center_ids, behind_center_ids);
            } else if ( sensor_lane - lane == -1 ) // left
            {
                check_ahead_behind(ahead_left_ids, behind_left_ids);
            } else if ( sensor_lane - lane == 1 ) // right
            {
                check_ahead_behind(ahead_right_ids, behind_right_ids);
            }
          }

          auto find_min_dist = [&](vector<int>& ids, int& id, double& speed, double& distance)
          {
            for (auto& i : ids)
            {
              double s_pos = s_pos_map[i];
              double diff = abs(s_pos - car_s);
              if (diff < distance)
              {
                id = i;
                distance = diff;
                speed = speed_map[id];
              }
            }
          };

          int ahead_center_id = -1;
          int ahead_left_id = -1;
          int ahead_right_id = -1;
          int behind_center_id = -1;
          int behind_left_id = -1;
          int behind_right_id = -1;

          double ahead_center_speed = -1;
          double ahead_left_speed = -1;
          double ahead_right_speed = -1;
          double behind_center_speed = -1;
          double behind_left_speed = -1;
          double behind_right_speed = -1;

          double ahead_center_dist = 9999;
          double ahead_left_dist = 9999;
          double ahead_right_dist = 9999;
          double behind_center_dist = 9999;
          double behind_left_dist = 9999;
          double behind_right_dist = 9999;

          // Find 6 cars' information {right, center, left} x {ahead, behind}
          find_min_dist(ahead_center_ids, ahead_center_id, ahead_center_speed, ahead_center_dist);
          find_min_dist(ahead_left_ids, ahead_left_id, ahead_left_speed, ahead_left_dist);
          find_min_dist(ahead_right_ids, ahead_right_id, ahead_right_speed, ahead_right_dist);
          find_min_dist(behind_center_ids, behind_center_id, behind_center_speed, behind_center_dist);
          find_min_dist(behind_left_ids, behind_left_id, behind_left_speed, behind_left_dist);
          find_min_dist(behind_right_ids, behind_right_id, behind_right_speed, behind_right_dist);

          bool lane_changed = false;

          // change lane and speed function.
          auto change_lane = [&](int direction)
          {
            lane += direction; // Change lane left.
            lane_changed = true;
          };
          auto change_speed = [&](double delta)
          {
            ref_vel = min(ref_vel + delta, max_speed);
          };

          if ( ahead_center_dist <= ahead_dist_limit )
          { // if car is ahead
            if ( ahead_left_dist > ahead_dist_limit && lane > left_most_lane )
            {
              if( (behind_left_dist > behind_dist_limit) && (ref_vel >= behind_left_speed) )
              { // if there is no car left and there is a left lane.
                change_lane(-1); // Change lane left.
              }
            }
            else if ( ahead_right_dist > ahead_dist_limit && lane != right_most_lane )
            {
              if ( (behind_right_dist > behind_dist_limit) && (ref_vel >= behind_right_speed) )
              { // if there is no car right and there is a right lane.
                change_lane(1); // Change lane right.
              }
            }

            if (!lane_changed)
            { // car is ahead and we can't change lane, so only slow down.
              change_speed(-speed_delta);
            }
          }
          else // There is no car ahead, so speed up
          {
            change_speed(speed_delta);
          }

          // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
          // later we will interpolate these waypoints with a spline and fill it in with more points that control speed.
          vector<double> ptsx;
          vector<double> ptsy;

          // reference x, y, yaw states
          // either we will reference the starting point as where the car is or at the previous paths end
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if previous size is almost empty, use the car as starting references
          if ( prev_size  < 2 )
          {
            //use two points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else
          {
            //redefine reference state as previous path end point
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            //use two points that make the path tangent to the previous path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // n Frenet add evenly 30m length points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s + 30, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for ( int i = 0; i < ptsx.size(); i++ )
          {
            //shift car reference angle to 0
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

          // create a spline object
          tk::spline s;

          // set points for the spline
          s.set_points(ptsx, ptsy);

          // Define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Start with all of the previous path points from last time
          for ( int i = 0; i < prev_size; i++ ) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);

          double x_add_on = 0;

          // Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
          for( int i = 1; i < 50 - prev_size; i++ ) {
            double N = target_dist / (.02 * ref_vel / 2.24); //2.24 for mph to m/s
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // rotate back to normal after rotating it earlier
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          json msgJson;

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