# Self-Driving Car Path Planning
An Udacity Self-Driving Car Engineer Nanodegree Program Project

---

## Introduction
In this project, the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. We are provided with the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car is to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

---

## Implementation
The objectives of the project were achieved through the following:
1. **A proportional velocity control:** Here, the target velocity of the ego-vehicle is set depending on the state of the vehicle. If there is no vehicle ahead of the ego-vehicle, the target velocity is set to 49.5 mph, which is just below the speed limit. However, if the ego-vehicle is close to another vehicle ahead of it, it adjusts the target velocity to the vehicle ahead of it and keeps a safety distance between them. The acceleration of the ego-vehicle to achieved using a proportional controller, as in; 
```cpp
  vel_control.Init(0.005, 0.0, 0.0);
...
...
            // target velocity control
            if (ego_vehicle.too_close)
            {
              // keeping lane
              speed_limit = ego_vehicle.other_car_vel;
            } else {
              speed_limit = 49.5;
            }

            double vel_error = ref_vel - speed_limit;
            vel_control.UpdateError(vel_error);
            double new_vel = vel_control.TotalError();
            ref_vel += new_vel;
```

2. Occupied lane avoidance logic: XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

3. Trajectory generation from a spline function: After a desired lane of the ego-vehicle has been set, We create `(x, y)` coordinate of waypoints ahead of the ego-vehicle using the corresponding spaced `(s,d)` Frenet coordinates. From these waypoints, we use a spline function to generate evenly spaced points for a smooth trajectory towards a desired horizon. The car is expected to visit each of these points every `0.02 secs`, so the number of the points are calculated by `N = double N = (target_dist/(0.02*ref_vel/2.24))` in order to travel at the desired reference velocity. In the implementation, 6 waypoints were created and a horizon of `30 m` was chosen to generate the desired trajectory.


See the original readme [here](https://github.com/toluwajosh/CarND-Path-Planning-Project/blob/master/Original_README.md)

---

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.


Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.