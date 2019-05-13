#include "planner.h"

void Planner::predict(double ego_s, int prev_size, vector<vector<double>> sensor_fusion) {
  isLeftLaneClear = true;
  isRightLaneClear = true;
  isThisLaneClear = true;
  
  // The goal is to analyze the sensor_fusion data to set the above flags correctly
  for ( int i = 0; i < sensor_fusion.size(); i++ ) {

    float d = sensor_fusion[i][6];

     // what lane is the other vehicle in
    int vehicle_lane;
    if (0.0 < d && d < 4.0) { 
      vehicle_lane = 0; 
      //std::cout << d << std::endl;
    } 
    else if (4.0 < d && d < 8.0) { vehicle_lane = 1; } 
    else if (8.0 < d && d < 12.0) { vehicle_lane = 2; }
    if (d < 0) { continue; }  // can ignore this vehicle
    //std::cout << "********" << std::endl;

    // predict what the vehicle's s-coordinate will be at next timestep
    double vehicle_speed_x = sensor_fusion[i][3];
    double vehicle_speed_y = sensor_fusion[i][4];
    double vehicle_s = sensor_fusion[i][5];
    double vehicle_speed_mag = sqrt( pow(vehicle_speed_x,2) + pow(vehicle_speed_y,2) );
    vehicle_s += ( (double)prev_size * 0.02 * vehicle_speed_mag );


    if (vehicle_lane == ego_lane) {
      if ( (vehicle_s > ego_s) && (vehicle_s - ego_s < 40.0) ) {   // Vehicle is in same lane and less than 30m ahead
        isThisLaneClear = false; 
      }
    } 
    else if (vehicle_lane == ego_lane - 1) {         // Vehicle is in lane directly to the left of the ego car
      if ((vehicle_s > ego_s - 25) && (ego_s + 50 > vehicle_s)) {
        isLeftLaneClear = false;
      }
    } 
    else if (vehicle_lane == ego_lane + 1) {         // Vehicle is in lane directly to the right of the ego car
      if ((vehicle_s > ego_s - 25) && (ego_s + 50 > vehicle_s)) {
        isRightLaneClear = false;
      }
    }
  }
}

void Planner::plan() {
  const double SPEED_LIMIT = 49.5;
  double MAX_ACC = 0.224;
  if (ego_target_speed < 0.5) { MAX_ACC = 0.15; }   //  Reduces the initial jerk

  if (isThisLaneClear) { 
    if (ego_target_speed < SPEED_LIMIT) { 
      ego_target_speed += MAX_ACC; 
    }
  } 
  else {
    if (ego_lane > 0 && isLeftLaneClear) {  
      ego_lane--;  
    } 
    else if (ego_lane < 2 && isRightLaneClear) { 
      ego_lane++; 
    } 
    else {
      ego_target_speed -= MAX_ACC;
    }
  }
  if (ego_target_speed > SPEED_LIMIT) { ego_target_speed = SPEED_LIMIT; }
}
