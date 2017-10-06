#include "Vehicle.h"
#include <iostream>
#include <vector>

using namespace std;

Vehicle::Vehicle() {}

Vehicle::~Vehicle() {}

void Vehicle::start(int lane, double speed_limit){
  this->lane = lane;
  this->speed_limit = speed_limit;
}

int Vehicle::next_lane(vector<vector<double>> sensor_fusion, int current_lane, double car_s, double end_path_s, int prev_size){
  
  // update lane
  lane = current_lane;


  // other car properties
  double other_car_dist = 10000000.0;
  double other_car_d = -1;
  int other_car_lane = -1;
  int other_car_id = -1;


  car_ahead = false;
  too_close = false;

  left_is_free = true;
  right_is_free = true;

  space_on_left = 10000.0;
  space_on_right = 10000.0;
  

  // for ego_vehicle:
  if (lane==0)
  {
    left_is_free = false;
  }
  if (lane==2)
  {
    right_is_free = false;
  }


    ///////////////////////////////////////////////////////////
    // check through the data (cars) from sensor fussion output
    for (int i = 0; i < sensor_fusion.size(); i++)
    {
      // find if a car is in my lane
      float d = sensor_fusion[i][6];

      // observe lane of car
      int lane_obsvd = fabs(d/4);

      // check the velocity of the car in my lane
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx+vy*vy);
      double check_car_s = sensor_fusion[i][5]; // the s coordinate of the car
      double check_car_d = sensor_fusion[i][6];
      // project where the car might be in the future
      check_car_s += ((double)prev_size*0.02*check_speed);
      double space = check_car_s - end_path_s;

      // want to track the nearest car to the ego car
      // double car_dist = fabs(check_car_s - car_s);
      double car_dist = check_car_s - car_s;

      // check cars within dangerous range
      // if car is in front
      if ((car_dist > 0) && (car_dist < 30))
      {
        if (lane_obsvd == (lane-1)){ // if an observed car is on the left side
            left_is_free = false;
          }
          if (lane_obsvd == (lane+1)){ // if an observed car is on the right side
            right_is_free = false;
          }
      } 
      else if ((car_dist < 0) && (car_dist > -20)) // if car is at the back
      {
        if (lane_obsvd == (lane-1)){ // if an observed car is on the left side
            left_is_free = false;
          }
          if (lane_obsvd == (lane+1)){ // if an observed car is on the right side
            right_is_free = false;
          }
      }

      if (car_dist < other_car_dist) {
        other_car_dist = car_dist;
        other_car_d = check_car_d;
        other_car_lane = other_car_d/4;
        other_car_id = i;
        other_car_vel = check_speed;
      }


      
      // if observed car is in my lane:
      if ((d<2+4*lane+2) && d > (2+4*lane-2))
      { 

        // of an observed car is ahead
        if ((check_car_s > car_s) && ((check_car_s-car_s)<(safety_space + 30))){
          car_ahead = true;

          if (lane_obsvd == (lane-1)){ // if an observed car is on the left side
            if (space < space_on_left) {space_on_left = space;}
            left_is_free = false;
          }
          if (lane_obsvd == (lane+1)){ // if an observed car is on the right side
            if (space < space_on_right) {space_on_right = space;}
            right_is_free = false;
          }
        }
        
        // if observed car is too close
        if ((check_car_s > car_s) && ((check_car_s-car_s)<safety_space))
        {
          
          // Do some logic here, lower reference velocity so we dont crash into the car infront of us,
          // could also flag to try to change lanes.
          // ref_vel = 29.5; //mph
          too_close = true;


          // cout << "\nleft_is_free: " << left_is_free
          //       << " right_is_free: " << right_is_free << endl;


        //   ///

        //   if ((lane==0) && right_is_free)
        //   {
        //       lane = 1;
        //   }
        //   else if (lane==1)
        //   {
        //     if (left_is_free && right_is_free)
        //     {
        //       // cout << "spaces >> left: " << space_on_left << 
        //       //         " right: " << space_on_right << endl;
        //       if (space_on_right == space_on_left)
        //       {
        //         lane += 1;
        //       } else {
        //         lane -= 1;
        //       }
        //     } else if (left_is_free) { 
        //       lane -= 1;
        //     } else if(right_is_free) {
        //       lane += 1;
        //     }
        //   }
        //   else if ((lane==2) && left_is_free)
        //   {
        //       lane = 1;
        //   }

        // cout << "changing to lane: " << lane << endl;
        // if (other_car_id != -1){
        //   cout << "other car: " << other_car_dist << " ,lane: " << other_car_lane << endl;
        // }

        // ///

        }
      }
    } // end of search through sensor fussion


    if (too_close) // car_ahead or too close
    {
      cout << "\nleft_is_free: " << left_is_free
          << " right_is_free: " << right_is_free << endl;
      if ((lane==0) && right_is_free)
          {
              lane = 1;
          }
          else if (lane==1)
          {
            if (left_is_free && right_is_free)
            {
              // cout << "spaces >> left: " << space_on_left << 
              //         " right: " << space_on_right << endl;
              if (space_on_right == space_on_left)
              {
                lane += 1;
              } else {
                lane -= 1;
              }
            } else if (left_is_free) { 
              lane -= 1;
            } else if(right_is_free) {
              lane += 1;
            }
          }
          else if ((lane==2) && left_is_free)
          {
              lane = 1;
          }
    }
    // else
    // {
    //   lane = current_lane;
    // }

    if (lane>2)
    {
      lane = 2;
    }

    // to prevent instantaneous lane change
    if (too_close){
      patience ++;
    }

    if (patience > 12){
      lane = current_lane;
      patience = 0;
    }
    // end of checking cars around ego-car
    ///////////////////////////////////////////////////////////////
    return lane;
}