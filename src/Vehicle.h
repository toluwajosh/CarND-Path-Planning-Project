#ifndef Vehicle_H
#define Vehicle_H
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>

using namespace std;

class Vehicle
{
  int lane = 1;
  int patience  = 6;
  double speed_limit = 49.5;

  double keep_lane_speed = 49.5;
  bool car_ahead = false;
  double other_car_dist = 10000000.0;
  double other_car_d = -1;
  int other_car_lane = -1;
  int other_car_id = -1;


  double safety_space = 50; // set a safety_space
  
  bool left_is_free = true;
  bool right_is_free = true;

  double space_on_left = 10000.0;
  double space_on_right = 10000.0;

  
public:
  Vehicle();
  ~Vehicle();

  bool too_close = false;

  // start car
  void start(int lane, double speed_limit);
  int next_lane(vector<vector<double>> sensor_fusion, int current_lane, double car_s, double end_path_s, int prev_size);
  
};




#endif /* Vehicle_H */