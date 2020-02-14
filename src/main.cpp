
#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "helpers.h"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;


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

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
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

    // start in Lane 1
    int lane = 1; //starting lane in simulator

    // Have a reference velocity to target
    double ref_vel = 2.5; // mph

    double max_vel = 49.5;

    h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy, &lane, &ref_vel, &max_vel](uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length,
        uWS::OpCode opCode) {
            // "42" at the start of the message means there's a websocket message event.
            // The 4 signifies a websocket message
            // The 2 signifies a websocket event
            //auto sdata = string(data).substr(0, length);
            //cout << sdata << endl;
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

                        // Sensor Fusion Data, a list of all other cars on the same side of the road.
                        auto sensor_fusion = j[1]["sensor_fusion"];
		
                        int prev_size = previous_path_x.size(); //previous path movement (amount of waypoints left that car have not gone through)
					    double frontcar_speed = 0.0; // speed of the car in front
                        // Using sensor data to prevent collision ***********
                        // Data given:
                       /*["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. */

                        if (prev_size > 0) {
                            car_s = end_path_s;
                        }

                        //flags:***
                        bool too_close = false;
                        bool sharp_brake =false;
                        bool change_lane = false;
                      
                        // all lanes initialise as clear first then it will be checked when we want to change the lane
                        bool middle_clear = true;
                        bool left_clear = true;
                        bool right_clear = true;
                        

                        int frontcar_lane = -999; // change this lane once we found a lane that is clear for lane changing

                    // Surrounding vehicles properties ******************************************************
                        for (int i = 0; i < sensor_fusion.size(); i++)
                        {
                            double check_car_s = sensor_fusion[i][5];
                            double vx = sensor_fusion[i][3];
                            double vy = sensor_fusion[i][4];
                            double check_speed = sqrt(vx * vx + vy * vy);

                            float d = sensor_fusion[i][6]; //lane check


                            // do a trajectory projection in terms of s (frenet):

                            check_car_s += (double)prev_size * 0.02 * check_speed;
                            double difference_s = check_car_s - car_s;

                            // Logic 1: Detecting Car in front of our car. Same lane and it is in front 

                            bool s_close = difference_s > 0 && difference_s < 30.0 ; // Is the car close and in front?

                            if ( s_close && get_current_lane(d) == lane) { // Is the close car in the same lane as us?


                                        too_close = true;  // we are close to the car in front of us
									    frontcar_speed = check_speed; //speed of the car in front
                                      
                                        if(difference_s <= 3.0)  sharp_brake = true; // need to brake more if we are too close (car suddenly coming into our lane)

                                        // Put a false flag to clearing flag for the lane that we are in (lane clear flag is used for switching lane so it does not make sense to switch to the lane 
                                        // we are already in
                                        if (lane == 0) left_clear = false;
                                        if (lane == 1) middle_clear = false;
                                        if (lane == 2) right_clear = false;
                                   
                            } 

                     
                          
                           // Logic 2: Check on whether there's car within a dangerous distance in other lane to change lane:*******

                            double to_clear_dist = 15.0; //distance that we happy to declare the lane as clear
                            bool lane_not_clear = fabs(difference_s) < to_clear_dist ;

                            if (lane_not_clear) {  // Within the scanning range, check for cars, if there is car in the lane then set clear flag to false

                              // Put a false flag to lanes where there are cars preventing us from switching lanes:
                              // S coordinate, Left: 0 , Middle: 1  , Right: 2

                              if (get_current_lane(d) == 1)  middle_clear = false;

                              if (get_current_lane(d) == 0)  left_clear = false;

                              if (get_current_lane(d) == 2)  right_clear = false;

                            
                            }
                            // Logic 2 ends ******************
 
                        } // Checked all vehicles on highway (for loop ends)



                        // Start up speed (lessen jerk) and also it will increase speed back once we are not close to the car in front
                        if (ref_vel <= max_vel && !too_close) {
                            // this statement also ensures that speed will always be around 49.5mph
                            ref_vel += .224;
                        }


                        // **************** which lane can we go to *******************************
                        // *We will only execute this logic check when we are to too close to the car in front of us 
                        // No point in changing lanes unless it is beneficial
                        if (too_close) {
                          

                            if (right_clear && lane != 0)  lane = 2;  // can only go right if you are in the middle	(prevent switching two lanes at once)

                            if (left_clear && lane != 2)   lane = 0;  // can only go left if you are in the middle  (prevent switching two lanes at once)					

                            if (middle_clear)  lane = 1;  

                           //***********************which lane to go ends ************************************

                           //****** Adapt speed when we are too close a car **********************************
     
                            if (ref_vel >= frontcar_speed) { // reduce to speed of the car in front

                                    ref_vel -= .224 ; // slow decrement (preventing jerking)
                                        
                            }
                         
                            if (sharp_brake) { // reduce speed further 
                                ref_vel -= .112; 
                            }

                        } //end if too_close condition is executed
                      


                        //********************* printing to check stuff ******************************

                        //cout << "Too close:" << too_close << endl;
                        //cout << "Lane:" << lane << endl;
                        //cout << "Front car lane:" << frontcar_lane << endl;
                        //cout << "Middle Cleared:" << middle_clear <<endl;
                        //cout << "Left Cleared:" << left_clear <<endl;
                        //cout << "Right Cleared:" << right_clear <<endl;


                        //*****************************************************


                        // ************ Creating waypoint


                        // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
                        // Later we will interpolate these waypoints with a spline and it in with more points that control needs.
                        vector<double> ptsx;
                        vector<double> ptsy;

                        // reference x,y, yaw states
                        // either we will reference the starting point as where the car is or at the previous paths end point
                        double ref_x = car_x;
                        double ref_y = car_y;
                        double ref_yaw = deg2rad(car_yaw);

                        // if previous size is almost empty, use the car as starting reference
                        if (prev_size < 2)
                        {
                            // Use two points tat make the path tangent to the car
                            double prev_car_x = car_x - cos(car_yaw);
                            double prev_car_y = car_y - sin(car_yaw);

                            ptsx.push_back(prev_car_x);
                            ptsx.push_back(car_x);

                            ptsy.push_back(prev_car_y);
                            ptsy.push_back(car_y);
                        }

                        // use the previous path's endpoint as starting reference
                        else {
                            // Use the last two points.
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

                        // In Frenet add evenly 30m spaced points ahead of the starting reference
                        vector<double> next_wp0 = getXY(car_s + 30, double(2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                        vector<double> next_wp1 = getXY(car_s + 60, double(2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                        vector<double> next_wp2 = getXY(car_s + 90, double(2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

                        ptsx.push_back(next_wp0[0]);
                        ptsx.push_back(next_wp1[0]);
                        ptsx.push_back(next_wp2[0]);

                        ptsy.push_back(next_wp0[1]);
                        ptsy.push_back(next_wp1[1]);
                        ptsy.push_back(next_wp2[1]);


                        for (int i = 0; i < ptsx.size(); i++)
                        {
                            // shift car reference angle to 0 degrees
                            double shift_x = ptsx[i] - ref_x;
                            double shift_y = ptsy[i] - ref_y;

                            ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
                            ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
                        }

                        // Create a spline.
                        tk::spline s;

                        // Set (x,y) points to the spline
                        s.set_points(ptsx, ptsy);

                        // Define the actual (x,y) points we will use for the planner
                        vector<double> next_x_vals;
                        vector<double> next_y_vals;
                        for (int i = 0; i < prev_size; i++)
                        {
                            next_x_vals.push_back(previous_path_x[i]);
                            next_y_vals.push_back(previous_path_y[i]);
                        }

                        // Calculate how to break up spline points so that we travel at our desired reference velocity
                        double target_x = 30.0;
                        double target_y = s(target_x);
                        double target_dist = sqrt(target_x * target_x + target_y * target_y);

                        double x_add_on = 0;

                        // Fill up the rest of our path planner after filling it with previous points, here we will always outputs 50 points
                        for (int i = 1; i < 50 - previous_path_x.size(); i++)
                        {

                            double N = target_dist / (0.02 * ref_vel / 2.24);
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


                        auto msg = "42[\"control\"," + msgJson.dump() + "]";

                        //this_thread::sleep_for(chrono::milliseconds(1000));
                        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                    }
                }
                else {
                    // Manual driving
                    std::string msg = "42[\"manual\",{}]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            }
        });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse* res, uWS::HttpRequest req, char* data,
        size_t, size_t) {
            const std::string s = "<h1>Hello world!</h1>";
            if (req.getUrl().valueLength == 1) {
                res->end(s.data(), s.length());
            }
            else {
                // i guess this should be done more gracefully?
                res->end(nullptr, 0);
            }
        });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
        });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
        char* message, size_t length) {
            ws.close();
            std::cout << "Disconnected" << std::endl;
        });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    }
    else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
