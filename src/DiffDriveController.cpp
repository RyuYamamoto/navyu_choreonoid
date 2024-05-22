#include <cmath>
#include <cnoid/SimpleController>

#include <cnoid/src/Body/ControllerIO.h>
#include <cnoid/src/Body/SimpleController.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <yaml-cpp/yaml.h>

#include <filesystem>

class DiffDriveController : public cnoid::SimpleController {
public:
  virtual bool configure(cnoid::SimpleControllerConfig *config) override {
    node_ = std::make_shared<rclcpp::Node>(config->controllerName());

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

    odometry_publisher_ =
        node_->create_publisher<nav_msgs::msg::Odometry>("odometry", 5);
    twist_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 1,
        std::bind(&DiffDriveController::callback_twist, this,
                  std::placeholders::_1));

    executor_ =
        std::make_unique<rclcpp::executors::StaticSingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { executor_->spin(); });

    // parameter loading
    std::filesystem::path param_file_path =
        ament_index_cpp::get_package_share_directory("navyu_choreonoid");
    param_file_path.append("config/diff_drive_controller_params.yaml");

    YAML::Node node = YAML::LoadFile(param_file_path.c_str());

    try {
      base_frame_ = node["config"]["base_frame"].as<std::string>();
      odom_frame_ = node["config"]["odom_frame"].as<std::string>();
      left_wheel_joint_ = node["config"]["left_wheel_joint"].as<std::string>();
      right_wheel_joint_ =
          node["config"]["right_wheel_joint"].as<std::string>();
      wheel_radius_ = node["config"]["wheel_radius"].as<double>();
      wheel_width_ = node["config"]["wheel_width"].as<double>();
    } catch (const YAML::TypedBadConversion<std::string> &e) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), e.msg.c_str());
      rclcpp::shutdown();
    }
    return true;
  }
  virtual bool initialize(cnoid::SimpleControllerIO *io) override {
    io_ = io;

    auto body = io->body();
    dt_ = io->timeStep();

    // get motor name from yaml
    wheels_[0] = body->joint(left_wheel_joint_);
    wheels_[1] = body->joint(right_wheel_joint_);
    for (int i = 0; i < 2; ++i) {
      auto wheel = wheels_[i];
      wheel->setActuationMode(JointVelocity);
      io->enableOutput(wheel);
    }
    return true;
  }
  virtual bool control() override {

    double dq_target[2];

    dq_target[0] = (twist_.linear.x - (twist_.angular.z * wheel_width_ / 2.0)) /
                   wheel_radius_;
    dq_target[1] = (twist_.linear.x + (twist_.angular.z * wheel_width_ / 2.0)) /
                   wheel_radius_;

    // update target velocity
    for (int i = 0; i < 2; ++i) {
      wheels_[i]->dq_target() = dq_target[i];
    }

    // update odometry
    const double v_l = dq_target[0] * wheel_radius_;
    const double v_r = dq_target[1] * wheel_radius_;
    const double v = (v_r + v_l) / 2.0;
    const double w = (v_r - v_l) / wheel_width_;

    const double delta_yaw = w * dt_;
    const double delta_x = v * std::cos(odometry_yaw_ + delta_yaw / 2.0) * dt_;
    const double delta_y = v * std::sin(odometry_yaw_ + delta_yaw / 2.0) * dt_;

    odometry_x_ += delta_x;
    odometry_y_ += delta_y;
    odometry_yaw_ += delta_yaw;

    odometry_yaw_ = normalized(odometry_yaw_);

    // odometry message
    nav_msgs::msg::Odometry odometry_msgs;
    odometry_msgs.header.frame_id = odom_frame_;
    odometry_msgs.header.stamp = get_stamp_msg_from_sec(io_->currentTime());
    odometry_msgs.child_frame_id = base_frame_;
    odometry_msgs.pose.pose.position.x = odometry_x_;
    odometry_msgs.pose.pose.position.y = odometry_y_;
    odometry_msgs.pose.pose.orientation =
        get_quaternion(0.0, 0.0, odometry_yaw_);
    odometry_publisher_->publish(odometry_msgs);

    // tf message
    geometry_msgs::msg::TransformStamped tf_msgs;
    tf_msgs.header = odometry_msgs.header;
    tf_msgs.child_frame_id = base_frame_;
    tf_msgs.transform.translation.x = odometry_x_;
    tf_msgs.transform.translation.y = odometry_y_;
    tf_msgs.transform.rotation = get_quaternion(0.0, 0.0, odometry_yaw_);
    tf_broadcaster_->sendTransform(tf_msgs);

    return true;
  }

  geometry_msgs::msg::Quaternion
  get_quaternion(const double roll, const double pitch, const double yaw) {
    tf2::Quaternion tf2_quat;
    tf2_quat.setRPY(roll, pitch, yaw);

    geometry_msgs::msg::Quaternion quaternion;
    quaternion.x = tf2_quat.x();
    quaternion.y = tf2_quat.y();
    quaternion.z = tf2_quat.z();
    quaternion.w = tf2_quat.w();

    return quaternion;
  }

  double normalized(const double radian) {
    double normalized_radian = radian;
    if (M_PI < radian)
      normalized_radian -= 2.0 * M_PI;
    else if (radian < -M_PI)
      normalized_radian += 2.0 * M_PI;
    return normalized_radian;
  }

  void callback_twist(const geometry_msgs::msg::Twist::SharedPtr msg) {
    twist_ = *msg;
  }

  builtin_interfaces::msg::Time get_stamp_msg_from_sec(double sec) {
    builtin_interfaces::msg::Time msg;
    msg.sec = int(sec);
    msg.nanosec = (sec - int(sec)) * 1000000000;
    return msg;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::unique_ptr<rclcpp::executors::StaticSingleThreadedExecutor> executor_;
  std::thread executor_thread_;
  std::mutex mutex_;

  geometry_msgs::msg::Twist twist_;
  double odometry_x_;
  double odometry_y_;
  double odometry_yaw_;

  cnoid::Link *wheels_[2];
  cnoid::SimpleControllerIO *io_;

  double dt_;
  std::string base_frame_;
  std::string odom_frame_;
  std::string left_wheel_joint_;
  std::string right_wheel_joint_;
  double wheel_radius_;
  double wheel_width_;
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(DiffDriveController)
