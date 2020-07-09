#include <control/visualization.h>

using namespace std;

// Visualization publishers for rviz

void Visualizer::publish_waypoints(Eigen::VectorXd waypoints_xs, Eigen::VectorXd waypoints_ys) {
  pcl::PointCloud<pcl::PointXYZI> waypoints;
  sensor_msgs::PointCloud2 pcl_msg;

  for (float i = 0.0; i < waypoints_xs.size(); i += 1) {
    pcl::PointXYZI p;
    p.x = waypoints_xs[i];
    p.y = waypoints_ys[i];
    p.z = 1.0;
    p.intensity = 0;
    waypoints.points.push_back(p);
  }

  pcl::toROSMsg(waypoints, pcl_msg);
  pcl_msg.header.frame_id = "/gps";
  waypoint_pub.publish(pcl_msg);
}

void Visualizer::publish_polyfit(Eigen::VectorXd K) {
  // f = K[3] * px0 * px0 + px0 + K[2] * px0 * px0 + K[1] * px0 + K[0];

  float max_time_step;
  if (!ros::param::get("/motion_planning/max_time_step", max_time_step)) {
      ROS_ERROR_STREAM("Visualizer - Error reading ROS config values from publish_polyfit");
      ros::shutdown();
  }

  pcl::PointCloud<pcl::PointXYZI> polyfit_pts;
  sensor_msgs::PointCloud2 pcl_msg;

  for (float x = 0.0; x <= max_time_step; x += 1.0) {
    double y_new = K[3] * x * x * x + K[2] * x * x + K[1] * x + K[0];

    pcl::PointXYZI p;
    p.x = x;
    p.y = y_new;
    p.z = 2.0;
    p.intensity = 0;
    polyfit_pts.points.push_back(p);
  }

  pcl::toROSMsg(polyfit_pts, pcl_msg);
  pcl_msg.header.frame_id = "/gps";
  polyfit_pub.publish(pcl_msg);
}

void Visualizer::publish_trajectory(vector<double> future_xs, vector<double> future_ys) {
  pcl::PointCloud<pcl::PointXYZI> trajectory_pts;
  sensor_msgs::PointCloud2 pcl_msg;

  for (int i = 0; i < future_xs.size(); i++) {
    pcl::PointXYZI p;
    p.x = future_xs[i];
    p.y = future_ys[i];
    p.z = 3.0;
    p.intensity = 0;
    trajectory_pts.points.push_back(p);
  }

  pcl::toROSMsg(trajectory_pts, pcl_msg);
  pcl_msg.header.frame_id = "/gps";
  trajectory_pub.publish(pcl_msg);
}
