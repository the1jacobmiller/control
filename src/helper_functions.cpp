#include "control/helper_functions.h"

void toRPY(const geometry_msgs::Quaternion &q, double &roll, double &pitch, double &yaw) {
  // roll (x-axis rotation)
  double sinr = +2.0 * (q.w * q.x + q.y * q.z);
  double cosr = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  roll = atan2(sinr, cosr);
  // pitch (y-axis rotation)
  double sinp = +2.0 * (q.w * q.y - q.z * q.x);
  if (fabs(sinp) >= 1) pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else pitch = asin(sinp);
  // yaw (z-axis rotation)
  double siny = +2.0 * (q.w * q.z + q.x * q.y);
  double cosy = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  yaw = atan2(siny, cosy);
}

double distance2DPoints(point point1, point point2) {
  return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));
}

Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); ++i) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); ++j) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  // auto Q = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV); // most accurate, slowest
  // auto Q = A.fullPivHouseholderQr();
  // auto Q = A.colPivHouseholderQr();
  auto Q = A.householderQr();

  auto result = Q.solve(yvals);
  // auto result = (A.transpose() * A).ldlt().solve(A.transpose() * yvals);

  return result;
}

Eigen::VectorXd linearfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals) {
  int num_pts = xvals.size();
  point p0 = point(xvals[0], yvals[0]);
  point pf = point(xvals[num_pts-1], yvals[num_pts-1]);

  double m = (pf.y - p0.y) / (pf.x - p0.x);
  double b = p0.y - m * p0.x;

  Eigen::VectorXd K(4);
  K[0] = b;
  K[1] = m;
  K[2] = 0.0;
  K[3] = 0.0;

  return K;
}
