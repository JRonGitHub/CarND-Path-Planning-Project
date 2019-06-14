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

  // ** Variable to be fed into the loop as part of the TODO **
  double targ_vel = 0.0;
  int lane = 1;
  int count = 0;

  h.onMessage([&count,&lane,&targ_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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

          //json msgJson;

          //vector<double> next_x_vals;
          //vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          // ** ATTENTION: There are a few variables fed into the cycle further up

          int prev_size = previous_path_x.size();

          //std::cout << "   got here  "  << std::endl;
          
          if (prev_size > 0)
          {
            car_s = end_path_s;
          }

          bool too_close = false;  // flag to break if cars get too close
          bool car_ahead = false;  // flag to initiate lane chg logic
          bool count_active = false;  // flag to start counting 
                                      // > smoothing lane changes to stay within rubric for acceleration and jerk

          double temp_trailing_speed = 49.5;
          //in case of changing lanes
          bool lane_0_unsafe = false;
          bool lane_1_unsafe = false;
          bool lane_2_unsafe = false;


          vector<int> lane_0_cars;
          vector<int> lane_1_cars;
          vector<int> lane_2_cars;

          struct Cars 
          {
            int id;
            double s;
            double d;
            double vel;
            double dist;
          };

          vector<Cars> cars_0;
          vector<Cars> cars_1;
          vector<Cars> cars_2;

          vector<Cars> too_close_vec;

          for (int i = 0; i < sensor_fusion.size(); i++)
          {
            float d = sensor_fusion[i][6];

            //sorting all cars according to the lane spaces using the frenet d 

            //lane 0
            if (d < (2+4*0+2) && d > (2+4*0-2))    
            {
              lane_0_cars.push_back(i);
            }
            //lane 1
            if (d < (2+4*1+2) && d > (2+4*1-2)) 
            {
              lane_1_cars.push_back(i);
            }
            //lane 2
            if (d < (2+4*2+2) && d > (2+4*2-2)) 
            {
              lane_2_cars.push_back(i);
            }

            if (lane_0_cars.empty() == false)
            {
              for (int c=0; c < lane_0_cars.size(); c++)
              {
                double vx = sensor_fusion[lane_0_cars[c]][3];
                double vy = sensor_fusion[lane_0_cars[c]][4];
                double check_speed = sqrt(vx*vx+vy*vy); //speed in m/s
                double check_car_s = sensor_fusion[lane_0_cars[c]][5];
                //calculate where cars s is in .02 seconds
                check_car_s += ((double)prev_size*.02*check_speed);

                //check if car in relevant space relative to my car
                if (  (check_car_s > (car_s-60)) && ((check_car_s - car_s)<120)  ) 
                {
                  Cars car;
                  car.id = sensor_fusion[lane_0_cars[c]][0];
                  car.s = check_car_s;
                  car.d = sensor_fusion[lane_0_cars[c]][6];
                  car.vel = check_speed;
                  car.dist =  (check_car_s - car_s); //if negative car behind my car

                  cars_0.push_back(car);  

                  //check if car is changing lanes and assign it already to next lane
                  /*
                  if (3.8 < car.d < 4)
                  {
                    cars_1.push_back(car);
                  }
                  */
                }
              }
            }

            if (lane_1_cars.empty() == false)
            {
              for (int c=0; c < lane_1_cars.size(); c++)
              {
                double vx = sensor_fusion[lane_1_cars[c]][3];
                double vy = sensor_fusion[lane_1_cars[c]][4];
                double check_speed = sqrt(vx*vx+vy*vy); //speed in m/s
                double check_car_s = sensor_fusion[lane_1_cars[c]][5];
                
                //calculate where cars s is in .02 seconds              
                check_car_s += ((double)prev_size*.02*check_speed);

                //check if car in relevant space relative to my car
                if (  (check_car_s > (car_s-60)) && ((check_car_s - car_s)<120)  ) 
                {
                  Cars car;
                  car.id = sensor_fusion[lane_1_cars[c]][0];
                  car.s = check_car_s;
                  car.d = sensor_fusion[lane_1_cars[c]][6];
                  car.vel = check_speed;
                  car.dist =  (check_car_s - car_s);  
                  cars_1.push_back(car);    

                  //check if car is changing lanes and assign it already to next lane
                  /*
                  if (4.1 < car.d < 4.2)
                  {
                    cars_0.push_back(car);
                  }

                  //check if car is changing lanes and assign it already to next lane
                  if (8.1 < car.d < 8.2)
                  {
                    cars_2.push_back(car);
                  }
                  */
                }        
              }
            }

            if (lane_2_cars.empty() == false)
            {
              for (int c=0; c < lane_2_cars.size(); c++)
              {
                double vx = sensor_fusion[lane_2_cars[c]][3];
                double vy = sensor_fusion[lane_2_cars[c]][4];
                double check_speed = sqrt(vx*vx+vy*vy); //speed in m/s
                double check_car_s = sensor_fusion[lane_2_cars[c]][5];
                //calculate where cars s is in .02 seconds
                check_car_s += ((double)prev_size*.02*check_speed);
                //std::cout << check_car_s << " after future estimate" << std::endl;

                //check if car in relevant space relative to my car
                if (  (check_car_s > (car_s-60)) && ((check_car_s - car_s)<120)  ) 
                {

                  Cars car;
                  car.id = sensor_fusion[lane_2_cars[c]][0];
                  car.s = check_car_s;
                  car.d = sensor_fusion[lane_2_cars[c]][6];
                  car.vel = check_speed;
                  car.dist =  (check_car_s - car_s);  

                  cars_2.push_back(car);

                  //check if car is changing lanes and assign it already to next lane
                  /*
                  if (8 < car.d < 8.2)
                  {
                    cars_1.push_back(car);
                  }
                  */
                }            
              }
            }
          }


          //get closest cars in front of my car
          double closest_dist = 9999;
          vector<Cars> closest_cars_0;
          if (cars_0.empty() == false)
          {
            for (int i = 0; i < cars_0.size(); i++ )
            {
              if (cars_0[i].dist < closest_dist && cars_0[i].dist >= 0)
              {
                closest_dist = cars_0[i].dist;
                closest_cars_0.push_back(cars_0[i]);  // the closest_cars_0.back() is closest in front for lane 0
              }
            }
          }

          double closest_dist_ = 9999;
          vector<Cars> closest_cars_1;
          if (cars_1.empty() == false)
          {
            for (int i = 0; i < cars_1.size(); i++ )
            {
              if (cars_1[i].dist < closest_dist_ && cars_1[i].dist >= 0)
              {
                closest_dist_ = cars_1[i].dist;
                closest_cars_1.push_back(cars_1[i]);  // the closest_cars_1.back() is closest in front for lane 1
              }
            }
          }

          double closest_dist_2 = 9999;
          vector<Cars> closest_cars_2;
          if (cars_2.empty() == false)
          {
            for (int i = 0; i < cars_2.size(); i++ )
            {
              if (cars_2[i].dist < closest_dist_2 && cars_2[i].dist >= 0)
              {
                closest_dist_2 = cars_2[i].dist;
                closest_cars_2.push_back(cars_2[i]);  // the closest_cars_2.back() is closest in front for lane 2
              }
            }
          }
          std::cout << targ_vel << std::endl;
          std::cout << lane << std::endl;

          //get closest cars behind my car
          double closest_behind = -9999;
          vector<Cars> closest_cars_behind_0;
          if (cars_0.empty() == false)
          {
            for (int i = 0; i < cars_0.size(); i++ )
            {
              if (cars_0[i].dist > closest_behind && cars_0[i].dist < 0)  //minus values
              {
                closest_behind = cars_0[i].dist;
                closest_cars_behind_0.push_back(cars_0[i]);  // the closest_cars_behind_0.back() is closest behind for lane 0
              }
            }
          }

          double closest_behind_ = -9999;
          vector<Cars> closest_cars_behind_1;
          if (cars_1.empty() == false)
          {
            for (int i = 0; i < cars_1.size(); i++ )
            {
              if (cars_1[i].dist > closest_behind_ && cars_1[i].dist < 0)
              {
                closest_behind_ = cars_1[i].dist;
                closest_cars_behind_1.push_back(cars_1[i]);  // the closest_cars_behind_1.back() is closest behind for lane 1
              }
            }
          }

          double closest_behind_2 = -9999;
          vector<Cars> closest_cars_behind_2;
          if (cars_2.empty() == false)
          {
            for (int i = 0; i < cars_2.size(); i++ )
            {
              if (cars_2[i].dist > closest_behind_2 && cars_2[i].dist < 0)
              {
                closest_behind_2 = cars_2[i].dist;
                closest_cars_behind_2.push_back(cars_2[i]);  // the closest_cars_behind_2.back() is closest behind for lane 2
              }
            }
          }

          //find most immediate space in lanes and add a costfunction
          double cost_0 = 0;
          double cost_1 = 0;
          double cost_2 = 0;

          //120 meters into the front cars are taken into account > the more dist there is the less the cost
          
          //if closest_cars_0 empty > segmentation fault error to be avoided plus best scenario! > cost = 0
          if (closest_cars_0.empty() == false)
          {
            cost_0 = 120 - (closest_cars_0.back().s - car_s);
            if (closest_cars_0.back().s > (car_s-2) && closest_cars_0.back().s < (car_s+5) || (closest_cars_0.back().s < (car_s+25) &&  closest_cars_0.back().vel < targ_vel) )
            {
              lane_0_unsafe = true;
            }
            if (lane == 0) // if closest_cars_0 not empty and lane == 0 there is a car in front of my car
            {
              car_ahead = true;
              if (closest_cars_0.back().dist < 30 && closest_cars_0.back().dist >= 0)
              {
                too_close = true;
                too_close_vec.push_back(closest_cars_0.back());
              }
            }
          }
          if (closest_cars_1.empty() == false)
          {
            cost_1 = 120 - (closest_cars_1.back().s - car_s);  
            if (closest_cars_1.back().s > (car_s-2) && closest_cars_1.back().s < (car_s+5) || (closest_cars_1.back().s < (car_s+25) &&  closest_cars_1.back().vel < targ_vel) )
            {
              lane_1_unsafe = true;
            }
            if (lane == 1) // if closest_cars_1 not empty and lane == 1 there is a car in front of my car
            {
              car_ahead = true;
              if (closest_cars_1.back().dist < 30 && closest_cars_1.back().dist >= 0)
              {
                too_close = true;
                too_close_vec.push_back(closest_cars_1.back());
              }
            }
          }
          if (closest_cars_2.empty() == false)
          {
            cost_2 = 120 - (closest_cars_2.back().s - car_s); 
            if (closest_cars_2.back().s > (car_s-2) && closest_cars_2.back().s < (car_s+5) || (closest_cars_2.back().s < (car_s+25) &&  closest_cars_2.back().vel < targ_vel) )
            {
              lane_2_unsafe = true;
            }
            if (lane == 2) // if closest_cars_2 not empty and lane == 2 there is a car in front of my car
            {
              car_ahead = true;
              if (closest_cars_2.back().dist < 30 && closest_cars_2.back().dist >= 0)
              {
                too_close = true;
                too_close_vec.push_back(closest_cars_2.back());
              }
            }
          }

          //60 meters into the back we take cars into account > the more dist there is the less the cost
          if (closest_cars_behind_0.empty() == false)
          {
            if (closest_cars_behind_0.back().s > (car_s-5) && closest_cars_behind_0.back().s < (car_s+2)) 
            {
              lane_0_unsafe = true;
            }
            if ((closest_cars_behind_0.back().dist > - 30 ) && closest_cars_behind_0.back().vel > targ_vel )
            {
              lane_0_unsafe = true;
            }
          }
          if (closest_cars_behind_1.empty() == false)
          {
            if (closest_cars_behind_1.back().s > (car_s-5) && closest_cars_behind_1.back().s < (car_s+2))
            {
              lane_1_unsafe = true;
            }
            if ((closest_cars_behind_1.back().dist > - 30 ) && closest_cars_behind_1.back().vel > targ_vel) //(targ_vel + targ_vel*1/10) )
            {
              lane_1_unsafe = true;
            }
          }
          if (closest_cars_behind_2.empty() == false)
          {
            if (closest_cars_behind_2.back().s > (car_s-5) && closest_cars_behind_2.back().s < (car_s+2))
            {
              lane_2_unsafe = true;
            }
            if ((closest_cars_behind_2.back().dist > - 30 ) && closest_cars_behind_2.back().vel > targ_vel )
            {
              lane_2_unsafe = true;
            }
          }
          

          // LANE CHANGE LOGIC:
          
          std::cout << " cost_0 " << cost_0 << "    cost_1 " << cost_1 << "     cost_2 " << cost_2 << std::endl;

          if (car_d < (2+4*lane+2) && car_d > (2+4*lane -2))
          {
          //change lanes to avoid slow cars ahead (if its safe only)
            if (car_ahead == true)
            {
              if (lane == 0)
              {
                if (lane_1_unsafe == false && cost_0 > cost_1)
                {
                  lane = 1;
                  count_active = true;
                }
                else if (lane_1_unsafe == false && cost_2 < cost_1 && cost_2 < cost_0)
                {
                  lane = 1;
                  count_active = true;
                }
              }

              else if (lane == 1)
              {

                if (lane_0_unsafe == false && lane_2_unsafe == false)
                {
                  if (count_active == false)
                  {
                    if (cost_0 > cost_2 && cost_2 < cost_1)
                    {
                      lane = 2;
                    }
                    else if (cost_2 > cost_0 && cost_0 < cost_1)
                    {
                      lane = 0;
                    }
                    else if (cost_2 == cost_0 && cost_0 < cost_1)
                    {
                      lane = 0;   //theoretially it would be the faster lane in most countries
                    }
                  }
                }

                else if (lane_0_unsafe == false && cost_0 < cost_1)
                {
                  if (count_active == false)
                  {
                    lane = 0;
                  }
                }
                else if (lane_2_unsafe == false && cost_2 < cost_1)
                {
                  if (count_active == false)
                  {
                    lane = 2;
                  }
                }

              }

              else if (lane == 2)
              {
                if (lane_1_unsafe == false && cost_2 > cost_1)
                {
                  lane = 1;
                  count_active = true;
                }
                else if (lane_1_unsafe == false && cost_0 < cost_1 && cost_0 < cost_2)
                {
                  lane = 1;
                  count_active = true;
                }
              }
      
            }
          }  

          //count add on in order to not violate total acceleration or jerk rubric points while changing 2 lanes
          //or while going back and forth between lanes
          //count if active and count below 50, count 0 if count not active 
          if (count_active == true && count < 49)
          {
            count += 1;
          }
          else if (count = 49)   //count 50 (which is theoretically next time code is run) equals 1 second if code runs through every 0.02 seconds
          {
            count = 0;
            count_active = false;
          }

          // LANE CHG LOGIC END

             
          //break if car gets too close, trying to trail if possible, to not always drive fast, slow, fast, slow
          if (too_close == true)
          {
            
            //std::cout << "  if (too_close)"  << std::endl;
            if (too_close_vec.empty() == false)
            {
              temp_trailing_speed = too_close_vec.back().vel;
              //std::cout << "  if (too_close) 2"  << std::endl;
              if (temp_trailing_speed < targ_vel)
              {
                //if car in front too slow, slow down as much as possible within rubric points
                if ((targ_vel - .5) > temp_trailing_speed)
                {
                  targ_vel -= .5;
                  std::cout << temp_trailing_speed << " temp_trailing_speed " << std::endl;
                  std::cout << "if ((targ_vel - .5) > temp_trailing_speed)" << std::endl;
                }
                //if car is in front is not that much slower any more, only slow down enough to not exceed trailing speed
                else if ((too_close_vec.back().s - car_s) > temp_trailing_speed/2 && temp_trailing_speed <= 49.5) //in german traffic schools the rule of thumb is to keep a distance of half the amount of speed in meters to the front (not making a conversion between mph and kmh here)
                {
                  targ_vel = temp_trailing_speed;
                  std::cout  << " else if (too_close_vec[-1].s - car_s) > temp_trailing_speed/2 -.5" << std::endl;
                }
                //for any cases we might not have thought of rather be extra safe
                else 
                {
                  targ_vel -= .5;
                  std::cout << " else statement -.5" << std::endl;
                }
              }
            
            
            //if close but speed slow enough get back to trailing speed
              else if (targ_vel < temp_trailing_speed && (targ_vel + .5) <= 49.5)
              {
                if (targ_vel + .5 <= temp_trailing_speed-.5)
                {
                  targ_vel += .5; 
                }
                else if ( targ_vel + .5 <= temp_trailing_speed && (too_close_vec.back().s - car_s) >= 5 ) //temp_trailing_speed < 15 &&
                {
                  targ_vel = temp_trailing_speed;
                }
                
              }
              
              else
              {
                targ_vel -= .5;
              }
            }
          }



          else if (targ_vel < 49)
          {
            targ_vel += .5;
          }

          else if (targ_vel < 49.5)
          {
            targ_vel += .224;
          }

          //Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m 
          //later we will interpolate these waypoints with a spline and fill it in with more points that control 
          vector<double>ptsx;
          vector<double>ptsy;

          //reference x, y yaw states
          //either we will reference the starting point as where the car is or at the previous paths and points
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          
          //if previous size is almost empty, use the car as starting reference
          if (prev_size < 2)
          {
            //Use two points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          //use the previous path's end point as starting reference
          else
          {
            //Redefine reference state as previous path end point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

            //Use two points that make the path tangent to the previous path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }


          //In Frenet add evenly 30m spaced points ahead of the starting references
          vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (int i = 0; i < ptsx.size(); i++)
          {
            //shift car reference angle to 0 degrees
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;

            ptsx[i] = (shift_x *cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x *sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));

          }
          //creating spline class with the code from source (all in spline.h): https://kluge.in-chemnitz.de/opensource/spline/
          tk::spline s;

          // set (x,y) points to the spline (anchorpoints)
          s.set_points(ptsx,ptsy);

          //Define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals; //defined above the ToDO already
          vector<double> next_y_vals;

          //Start with all of the previous path points from last time (filling up with previous points)
          for(int i = 0; i < previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          //Calculate how to break up spline points so that we travel at desired target/reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));  //Pythagoras

          double x_add_on = 0;

          //Fill up the rest of path planner after filling it with previous points, here we will always output 50 points
          for (int i = 1; i <= 50 - previous_path_x.size(); i++)
          {
            double N = (target_dist/(.02*targ_vel/2.24));  //from mph to m/s therefore division by 2.24  (2.24mph = 1.00137 m/s) //.02 for .02seconds controller update interval
            double x_point = x_add_on+(target_x)/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            //rotate back to normal after rotating it earlier  (shift and rotation -- inverse of conversion before)
            x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));

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