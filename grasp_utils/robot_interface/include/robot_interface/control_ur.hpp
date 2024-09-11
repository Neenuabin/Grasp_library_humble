#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_interface/control_base.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <vector>

// Default values for parameters
static const std::string DEFAULT_HOST = "192.168.0.5";
static const std::vector<std::string> DEFAULT_JOINTS = {
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
};
static const bool DEFAULT_SHUTDOWN_ON_DISCONNECT = true;


struct ProgArgs {
  std::string host;
  std::vector<std::string> joint_names;
  bool shutdown_on_disconnect;
};

class URControl : public ArmControlBase {
public:
  URControl(const std::string node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~URControl();

  virtual bool moveToTcpPose(double x, double y, double z, 
                             double alpha, double beta, double gamma, 
                             double vel, double acc) override;

  virtual bool moveToJointValues(const std::vector<double>& joint_values, double vel, double acc) override;

  virtual bool open(const double distance = 0) override;
  virtual bool close(const double distance = 0) override;

  bool urscriptInterface(const std::string command_script);

  virtual bool getTcpPose();
  virtual bool getJointValues();
  virtual bool publish();

  bool startLoop();

  void parseArgs();  // Parses command line arguments

private:
  std::vector<std::string> joint_names_;
  bool gripper_powered_up_;

  ProgArgs args_;  // Declare args_ here

  // Action client for controlling joint trajectories
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr trajectory_action_client_;
};

