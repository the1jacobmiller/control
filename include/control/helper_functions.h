#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H

#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Dense>
#include "control/data.h"
#include <geometry_msgs/Quaternion.h>

void toRPY(const geometry_msgs::Quaternion &q, double &roll, double &pitch, double &yaw);
double distance2DPoints(point point1, point point2);
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);
Eigen::VectorXd linearfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals);

#endif
