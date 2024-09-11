#include <robot_interface/control_ur.hpp>
#include <tf2_eigen/tf2_eigen.hpp> // Updated include based on warning
#include <chrono>  // Include chrono for time literals
using namespace std::chrono_literals;  // Use chrono literals like 1s

URControl::URControl(const std::string node_name, const rclcpp::NodeOptions & options)
: ArmControlBase(node_name, options), gripper_powered_up_(false)
{
  // Initialize joint names
  joint_names_ = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

  // Initialize action client for controlling joint trajectories
  trajectory_action_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
    shared_from_this(),  // Use the node directly
    "joint_trajectory_controller/follow_joint_trajectory");
}

URControl::~URControl()
{
  RCLCPP_INFO(this->get_logger(), "UR control interface shut down.");
}
bool URControl::moveToTcpPose(double x, double y, double z, 
                              double alpha, double beta, double gamma, 
                              double vel, double acc)
{
  RCLCPP_INFO(this->get_logger(), "Moving to TCP Pose...");

  // Convert euler angles around (x, y, z) to rotation vector using tf2
  tf2::Quaternion q;
  q.setRPY(alpha, beta, gamma);
  Eigen::Isometry3d pose_goal = Eigen::Translation3d(x, y, z)
    * Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX())
    * Eigen::AngleAxisd(beta, Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(gamma, Eigen::Vector3d::UnitZ());

  // Convert to joint positions (use inverse kinematics, or send directly to robot via joint control)
  std::vector<double> joint_values = { /* result from inverse kinematics */ };

  return moveToJointValues(joint_values, vel, acc);
}

bool URControl::moveToJointValues(const std::vector<double>& joint_values, double vel, double acc)
{
  RCLCPP_INFO(this->get_logger(), "Moving to joint values...");

  // Create a JointTrajectory message and populate it with the joint values
  trajectory_msgs::msg::JointTrajectory trajectory_msg;
  trajectory_msg.joint_names = joint_names_;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = joint_values;
  point.time_from_start = rclcpp::Duration::from_seconds(5.0);  // Set appropriate duration based on vel and acc

  trajectory_msg.points.push_back(point);

  // Send the trajectory to the action server
  auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();
  goal_msg.trajectory = trajectory_msg;

  auto goal_handle_future = trajectory_action_client_->async_send_goal(goal_msg);
  
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to send joint trajectory");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Joint trajectory sent successfully");
  return true;
}

bool URControl::open(const double distance)
{
  RCLCPP_INFO(this->get_logger(), "Opening gripper...");
  // Implement gripper control logic here, if applicable
  return true;
}

bool URControl::close(const double distance)
{
  RCLCPP_INFO(this->get_logger(), "Closing gripper...");
  // Implement gripper control logic here, if applicable
  return true;
}

bool URControl::urscriptInterface(const std::string command_script)
{
  // For URScript commands, use a ROS 2 service to communicate with the UR driver
  RCLCPP_INFO(this->get_logger(), "Sending URScript command...");
  return true;
}

void URControl::parseArgs()
{
  // Initialize parameter client
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);

  // Wait for the parameters service with a 1-second interval
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
  }

  // Get parameters with default values
  args_.host = parameters_client->get_parameter("host", DEFAULT_HOST);
  args_.joint_names = parameters_client->get_parameter("joint_names", DEFAULT_JOINTS);
  args_.shutdown_on_disconnect = parameters_client->get_parameter("shutdown_on_disconnect", DEFAULT_SHUTDOWN_ON_DISCONNECT);

  // Print parameters
  RCLCPP_INFO(this->get_logger(), "Host: %s", args_.host.c_str());
  std::stringstream ss;
  for (const auto &name : args_.joint_names) {
    ss << name << " ";
  }
  RCLCPP_INFO(this->get_logger(), "Joint Names: %s", ss.str().c_str());
  RCLCPP_INFO(this->get_logger(), "Shutdown on Disconnect: %s", args_.shutdown_on_disconnect ? "true" : "false");
}
bool URControl::startLoop()
{
  // No custom loop required in ROS 2
  RCLCPP_INFO(this->get_logger(), "Starting control loop (handled by ROS 2 driver)");

  return true;
}

bool URControl::getTcpPose()
{
  // The TCP pose can be obtained from the current robot state using the robot's
  // state feedback, which is handled by the Universal_Robots_ROS2_Driver.
  RCLCPP_INFO(this->get_logger(), "Getting TCP pose...");
  return true;
}

bool URControl::getJointValues()
{
  // The joint values can be obtained from the current robot state using the robot's
  // state feedback, which is handled by the Universal_Robots_ROS2_Driver.
  RCLCPP_INFO(this->get_logger(), "Getting joint values...");
  return true;
}

bool URControl::publish()
{
  RCLCPP_INFO(this->get_logger(), "Publishing joint states...");
  return true;
}

