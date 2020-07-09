#ifndef PATH_H
#define PATH_H

#include <iostream>     // std::cout
#include <fstream>      // std::ifstream
#include "control/data.h"
#include "control/helper_functions.h"
#include <ros/ros.h>

using namespace std;

class Path {
  public:
    vector<point> points;

    int findClosestPoint(point target_position);
    void clear();
};

#endif
