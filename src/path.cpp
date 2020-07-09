#include "control/path.h"

int Path::findClosestPoint(point target_position) {
  if (points.size() == 0) { ROS_ERROR_STREAM("Path - No target_path data found\n"); return -1; }
  static bool first_Iter = true;
  static int prev_closestIndex;

  int begin_check, max_check;
  if (first_Iter) { // look at all points
    begin_check = 0; first_Iter = false;
    max_check = points.size() - 1;
  }
  else { // predict next point based on previous point index
    begin_check = prev_closestIndex;
    max_check = prev_closestIndex + 20;
    if (max_check > points.size() - 1) max_check = points.size() - 1;
  }

  int closestIndex;
  double minimumDist = numeric_limits<double>::max();
  for (int i = begin_check; i < max_check; i++) {
    double dist = distance2DPoints(points[i], target_position);
    if (dist < minimumDist) { minimumDist = dist; closestIndex = i; }
  }
  prev_closestIndex = closestIndex;
  return closestIndex;
}

void Path::clear() {
  points.clear();
}
