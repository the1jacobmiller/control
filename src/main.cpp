#include <iostream>     // std::cout
#include <fstream>      // std::ifstream
#include <sys/shm.h>
#include <Eigen/Core>
#include <Eigen/QR>
#include "MPC.h"
#include "control/visualization.h"
#include "control/path.h"
#include "control/helper_functions.h"
#include "control/data.h"

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <dbw_mkz_msgs/SteeringReport.h>
#include <dbw_mkz_msgs/dbw_cmd.h>
#include <hmi/HMIScreenCmd.h>

using namespace std;

double target_speed;
double localizer_roll, localizer_pitch, localizer_yaw, vehicle_speed_ms, steering_wheel_angle, accel, delay_time;
point localizer_position;
Path target_path;

ros::Publisher command_pub, hmi_screen_pub;
ros::Timer mpc_timer;
Visualizer vis;

// Parameters
int num_waypoints;
double wheelbase = 2.8498;
double steering_ratio_ = 14.8;

bool stopping = false;

void read_configs() {
  if (!ros::param::get("/control/delay_time", delay_time)) {
    ROS_ERROR_STREAM("Failed to read ROS configs from main");
    ros::shutdown();
  }
}

// ---------------------------- Callbacks ------------------------------- //

void recvSteeringReport(const dbw_mkz_msgs::SteeringReport::ConstPtr &msg) {
  vehicle_speed_ms = msg->speed;
  steering_wheel_angle = msg->steering_wheel_angle;
}

void recvAccel(const std_msgs::Float64::ConstPtr &msg) {
  accel = msg->data;
}

void localizationCallback(const geometry_msgs::PoseStamped msg) {
  geometry_msgs::Pose pose = msg.pose;
  localizer_position = point(pose.position.x, pose.position.y);
  toRPY(pose.orientation, localizer_roll, localizer_pitch, localizer_yaw);
}

void stop_vehicle() {
  float decel_limit;
  if (!ros::param::get("/twist_controller/decel_limit", decel_limit)) {
      ROS_ERROR_STREAM("Main - Error reading ROS config values from stop_vehicle");
      ros::shutdown();
  }

  // publish control parameters
  dbw_mkz_msgs::dbw_cmd cmd;
	cmd.steering_cmd = 0.0; // determines steering_cmd - negative angle turns wheel to the right
  cmd.accel_cmd = -1.0 * decel_limit;
	command_pub.publish(cmd);

  ROS_WARN_STREAM("Main - Stopping vehicle!!");
}

void trajectoryCallback(const visualization_msgs::Marker::ConstPtr &msg) {
  if (msg->points.size() <= 1) {
    stopping = true;
    stop_vehicle();
    return;
  }
  stopping = false;

  Path new_path;
  for (auto pt : msg->points) {
    point new_point(pt.x, pt.y);
    new_path.points.push_back(new_point);
  }
  target_path = new_path;
  num_waypoints = target_path.points.size();
}

void speedCallback(const std_msgs::Float32::ConstPtr &msg) {
  float max_speed;
  if (!ros::param::get("/path_planning/max_speed", max_speed)) {
    ROS_ERROR_STREAM("Failed to read ROS configs from speedCallback");
  }

  target_speed = msg->data;

  hmi::HMIScreenCmd hmi_screen_cmd;
  hmi_screen_cmd.screen_key = 3;
  hmi_screen_cmd.line1 = "Max Spd: " + to_string(int(max_speed));
  hmi_screen_cmd.line2 = "Target Spd: " + to_string(int(target_speed));
  hmi_screen_pub.publish(hmi_screen_cmd);
}

void control_callback(const ros::TimerEvent& event) {
  static MPC mpc_;

  if (stopping) return;

  // set the current state
  double px = localizer_position.x;
  double py = localizer_position.y;
  double psi = localizer_yaw;
  double v = vehicle_speed_ms;
  double delta = (steering_wheel_angle / steering_ratio_);
  double a = accel;

  if (px == 0.0 || py == 0.0 || psi == 0.0) {
    return;
  }

  // convert waypoints to vehicle's reference frame
  Eigen::VectorXd waypoints_xs(num_waypoints);
  Eigen::VectorXd waypoints_ys(num_waypoints);
  int cpIndex = target_path.findClosestPoint(localizer_position);

  if (cpIndex == -1) return;

  int index = 0;
  for (int i = 0; i < num_waypoints; i++) {
    double dx = target_path.points[i].x - px;
    double dy = target_path.points[i].y - py;

    // waypoints_ys[index] = (dy - dx * tan(psi)) / (cos(psi) + tan(psi)*sin(psi));
    // waypoints_xs[index] = (dx + waypoints_ys[index] * sin(psi)) / cos(psi);

    double minus_psi = 0.0 - psi;
    waypoints_xs[i] = dx * cos(minus_psi) - dy * sin(minus_psi);
    waypoints_ys[i] = dx * sin(minus_psi) + dy * cos(minus_psi);

    index++;
  }
  vis.publish_waypoints(waypoints_xs, waypoints_ys);

  // fit polynomial to waypoints
  int order = 3;
  Eigen::VectorXd K = polyfit(waypoints_xs, waypoints_ys, 3);
  // Eigen::VectorXd K = linearfit(waypoints_xs, waypoints_ys);
  ROS_INFO_STREAM("K: " << K[0] << ", " << K[1] << ", " << K[2] << ", " << K[3]);
  if(isnan(K[0]) || isnan(K[1]) || isnan(K[2]) || isnan(K[3]) ||
    (K[0] == 0.0 && K[1] == 0.0 && K[2] == 0.0 && K[3] == 0.0)) {
    ROS_ERROR_STREAM("Main - Failed to fit polynomial");
    K = linearfit(waypoints_xs, waypoints_ys);
  }
  vis.publish_polyfit(K);

  // calculate current error estimates
  // cte - cross track error
  // f = K[3] * px0 * px0 + px0 + K[2] * px0 * px0 + K[1] * px0 + K[0];
  double cte = K[0];

  // epsi - orientation error
  // f' = 3.0 * K[3] * px0 * px0 + 2.0 * K[2] * px0 + K[1]
  double epsi = -atan(K[1]);

  // calculate current delayed state
  double current_px = 0.0 + v * delay_time;
  double current_py = 0.0;
  double current_psi = 0.0 + v * tan(delta) / wheelbase * delay_time;
  double current_v = v + a * delay_time;
  double current_cte = cte + v * sin(epsi) * delay_time;
  double current_epsi = epsi + v * tan(delta) / wheelbase * delay_time;

  Eigen::VectorXd state(6);
  state << current_px, current_py, current_psi, current_v, current_cte, current_epsi;
  // state << 0.0, 0.0, 0.0, v, cte, epsi;
  ROS_INFO_STREAM("Current state: " << current_px << ", " << current_py << ", " << current_psi << ", " << current_v << ", " << current_cte << ", " << current_epsi);
  mpc_.solve(state, K);

  dbw_mkz_msgs::dbw_cmd cmd;
	cmd.steering_cmd = mpc_.steer; // determines steering_cmd - negative angle turns wheel to the right
  cmd.accel_cmd = mpc_.throttle;
  vis.publish_trajectory(mpc_.future_xs, mpc_.future_ys);
	command_pub.publish(cmd);

  ROS_WARN_STREAM("Steering Cmd: " << mpc_.steer);
  ROS_WARN_STREAM("Accel Cmd: " << mpc_.throttle << "\n");

  hmi::HMIScreenCmd hmi_screen_cmd;
  hmi_screen_cmd.screen_key = 0;
  hmi_screen_cmd.line1 = "Steering: " + to_string(mpc_.steer);
  hmi_screen_cmd.line2 = "Accel: " + to_string(mpc_.throttle);
  hmi_screen_pub.publish(hmi_screen_cmd);
}

int main(int argc, char **argv) {
  // initialize ros node
  ros::init(argc, argv, "main_node");
  ros::NodeHandle n;
  ros::Rate r(30);

  read_configs();

  // Publishers
  command_pub = n.advertise<dbw_mkz_msgs::dbw_cmd>("/dbw_cmd", 1); // publishes to TwistControllerNode

  // Visualization Publishers
  vis.polyfit_pub = n.advertise<sensor_msgs::PointCloud2>("/control/polyfit", 1);
  vis.waypoint_pub = n.advertise<sensor_msgs::PointCloud2>("/control/waypoints", 1);
  vis.trajectory_pub = n.advertise<sensor_msgs::PointCloud2>("/control/predicted_trajectory", 1);
  hmi_screen_pub = n.advertise<hmi::HMIScreenCmd>("/hmi/screen_cmd", 1);

  // Subscribers
  ros::Subscriber sub_trajectory = n.subscribe("/motion_planning/trajectory", 1, trajectoryCallback);
  ros::Subscriber sub_speed = n.subscribe("/motion_planning/speed", 1, speedCallback);
  ros::Subscriber sub_localization = n.subscribe("/gnss_pose", 1, localizationCallback);
  ros::Subscriber sub_steering_report = n.subscribe("/vehicle/steering_report", 1, recvSteeringReport);
  ros::Subscriber sub_accel = n.subscribe("/vehicle/filtered_accel", 1, recvAccel);

  mpc_timer = n.createTimer(ros::Duration(dt), control_callback);

  ros::MultiThreadedSpinner spinner(0); // use one thread for each CPU core
  spinner.spin();

}
