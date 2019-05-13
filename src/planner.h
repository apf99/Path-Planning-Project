#ifndef planner_h
#define planner_h

#include <vector>
#include <string>

using namespace std;

class Planner {

  public:
    int ego_lane = 1;
    double ego_target_speed;
    bool isLeftLaneClear;
    bool isRightLaneClear;
    bool isThisLaneClear;
  
    // determine what options are available to the planner based on sensor_fusion
    void predict(double s, int prev_size, vector<vector<double>> sensor_fusion);

    // Make a steering and speed decision based on the options available
    void plan();
};

#endif /* planner_h */    
