#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2/utils.h>

class MpcPathFollower {
public:
  MpcPathFollower(ros::NodeHandle &nh, ros::NodeHandle &pnh) {
    pnh.param<std::string>("path_topic", path_topic_, "/searched_path");
    pnh.param<std::string>("odom_topic", odom_topic_, "/gazebo_odom");
    pnh.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");

    pnh.param("control_rate", control_rate_, 10.0);
    pnh.param("horizon_steps", horizon_steps_, 10);
    pnh.param("dt", dt_, 0.1);
    pnh.param("lambda", lambda_, 0.1);

    pnh.param("max_vx", max_vx_, 1.0);
    pnh.param("max_vy", max_vy_, 1.0);
    pnh.param("max_w", max_w_, 1.0);

    path_sub_ = nh.subscribe(path_topic_, 1, &MpcPathFollower::OnPath, this);
    odom_sub_ = nh.subscribe(odom_topic_, 1, &MpcPathFollower::OnOdom, this);
    cmd_pub_ = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);

    const double timer_dt = (control_rate_ > 0.0) ? (1.0 / control_rate_) : 0.1;
    control_timer_ = nh.createTimer(ros::Duration(timer_dt),
                                    &MpcPathFollower::OnTimer, this);
  }

private:
  struct RefPoint {
    double x;
    double y;
    double yaw;
  };

  void OnPath(const nav_msgs::Path::ConstPtr &msg) {
    path_ = msg->poses;
    has_path_ = !path_.empty();
  }

  void OnOdom(const nav_msgs::Odometry::ConstPtr &msg) {
    odom_ = *msg;
    has_odom_ = true;
  }

  void OnTimer(const ros::TimerEvent &) {
    if (!has_path_ || !has_odom_ || path_.empty()) {
      return;
    }

    geometry_msgs::Twist cmd;
    if (!ComputeMpcCommand(&cmd)) {
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.angular.z = 0.0;
    }

    cmd_pub_.publish(cmd);
  }

  bool ComputeMpcCommand(geometry_msgs::Twist *cmd) {
    if (horizon_steps_ < 1 || dt_ <= 0.0) {
      return false;
    }

    const auto &pose = odom_.pose.pose;
    const double x0 = pose.position.x;
    const double y0 = pose.position.y;
    const double yaw0 = tf2::getYaw(pose.orientation);

    const size_t nearest = FindNearestIndex(x0, y0);
    std::vector<RefPoint> refs;
    BuildReferenceTrajectory(nearest, horizon_steps_, &refs);

    if (refs.size() < static_cast<size_t>(horizon_steps_ + 1)) {
      return false;
    }

    std::vector<double> yaw_ref(refs.size());
    for (size_t i = 0; i < refs.size(); ++i) {
      yaw_ref[i] = refs[i].yaw;
    }
    UnwrapYaw(&yaw_ref);

    const int N = horizon_steps_;
    Eigen::MatrixXd A(2 * N, 2 * N);
    Eigen::VectorXd b(2 * N);
    A.setZero();
    b.setZero();

    for (int k = 0; k < N; ++k) {
      const double dx = refs[k + 1].x - x0;
      const double dy = refs[k + 1].y - y0;
      b(2 * k) = dx;
      b(2 * k + 1) = dy;

      for (int i = 0; i <= k; ++i) {
        const double cy = std::cos(yaw_ref[i]);
        const double sy = std::sin(yaw_ref[i]);
        A(2 * k, i) = dt_ * cy;
        A(2 * k, i + N) = -dt_ * sy;
        A(2 * k + 1, i) = dt_ * sy;
        A(2 * k + 1, i + N) = dt_ * cy;
      }
    }

    Eigen::MatrixXd reg = lambda_ * Eigen::MatrixXd::Identity(2 * N, 2 * N);
    Eigen::MatrixXd H = A.transpose() * A + reg;
    Eigen::VectorXd g = A.transpose() * b;

    Eigen::VectorXd u = H.ldlt().solve(g);

    if (u.size() != 2 * N) {
      return false;
    }

    const double vx = Clamp(u(0), -max_vx_, max_vx_);
    const double vy = Clamp(u(N), -max_vy_, max_vy_);

    Eigen::MatrixXd Ayaw(N, N);
    Eigen::VectorXd byaw(N);
    Ayaw.setZero();
    byaw.setZero();
    for (int k = 0; k < N; ++k) {
      const double dyaw = yaw_ref[k + 1] - yaw0;
      byaw(k) = dyaw;
      for (int i = 0; i <= k; ++i) {
        Ayaw(k, i) = dt_;
      }
    }
    Eigen::MatrixXd Hyaw =
        Ayaw.transpose() * Ayaw + lambda_ * Eigen::MatrixXd::Identity(N, N);
    Eigen::VectorXd gyaw = Ayaw.transpose() * byaw;
    Eigen::VectorXd uw = Hyaw.ldlt().solve(gyaw);

    const double w = (uw.size() > 0) ? Clamp(uw(0), -max_w_, max_w_) : 0.0;

    cmd->linear.x = vx;
    cmd->linear.y = vy;
    cmd->linear.z = 0.0;
    cmd->angular.x = 0.0;
    cmd->angular.y = 0.0;
    cmd->angular.z = w;

    return true;
  }

  size_t FindNearestIndex(double x, double y) const {
    size_t best_index = 0;
    double best_dist = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < path_.size(); ++i) {
      const auto &p = path_[i].pose.position;
      const double dx = p.x - x;
      const double dy = p.y - y;
      const double dist = dx * dx + dy * dy;
      if (dist < best_dist) {
        best_dist = dist;
        best_index = i;
      }
    }

    return best_index;
  }

  void BuildReferenceTrajectory(size_t start_index, int horizon,
                                std::vector<RefPoint> *refs) const {
    refs->clear();
    refs->reserve(static_cast<size_t>(horizon + 1));

    const size_t last_index = path_.empty() ? 0 : path_.size() - 1;
    for (int k = 0; k <= horizon; ++k) {
      const size_t idx =
          std::min(start_index + static_cast<size_t>(k), last_index);
      const auto &pose = path_[idx].pose;
      RefPoint ref;
      ref.x = pose.position.x;
      ref.y = pose.position.y;
      ref.yaw = tf2::getYaw(pose.orientation);
      refs->push_back(ref);
    }
  }

  void UnwrapYaw(std::vector<double> *yaw) const {
    if (!yaw || yaw->empty()) {
      return;
    }
    for (size_t i = 1; i < yaw->size(); ++i) {
      const double delta = NormalizeAngle((*yaw)[i] - (*yaw)[i - 1]);
      (*yaw)[i] = (*yaw)[i - 1] + delta;
    }
  }

  double NormalizeAngle(double angle) const {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  double Clamp(double value, double min_value, double max_value) const {
    return std::max(min_value, std::min(max_value, value));
  }

  ros::Subscriber path_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher cmd_pub_;
  ros::Timer control_timer_;

  std::vector<geometry_msgs::PoseStamped> path_;
  nav_msgs::Odometry odom_;
  bool has_path_{false};
  bool has_odom_{false};

  std::string path_topic_;
  std::string odom_topic_;
  std::string cmd_vel_topic_;

  double control_rate_{10.0};
  int horizon_steps_{10};
  double dt_{0.1};
  double lambda_{0.1};

  double max_vx_{1.0};
  double max_vy_{1.0};
  double max_w_{1.0};
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "mpc_path_follower");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  MpcPathFollower follower(nh, pnh);

  ros::spin();
  return 0;
}
