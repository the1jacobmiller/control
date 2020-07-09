#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include "control/data.h"
#include "control/path.h"

using namespace std;

class Visualizer {
  public:
    ros::Publisher polyfit_pub;
    ros::Publisher waypoint_pub;
    ros::Publisher trajectory_pub;
    ros::Publisher vsi_path_pub;

    void publish_waypoints(Eigen::VectorXd waypoints_xs, Eigen::VectorXd waypoints_ys);
    void publish_polyfit(Eigen::VectorXd K);
    void publish_trajectory(vector<double> future_xs, vector<double> future_ys);
};

#endif
