#pragma once

// ROS
#include <ros/ros.h>

// System
#include <memory>

// ROS msgs
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/KinematicSolverInfo.h>
#include <moveit_msgs/MoveItErrorCodes.h>

// MoveIt
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_state/robot_state.h>

namespace FABRIK_kinematics_plugin
{
/**
 * @brief Specific implementation of kinematics using ROS service calls to communicate with
   external IK solvers. This version can be used with any robot. Supports non-chain kinematic groups
 */
class FABRIKKinematicsPlugin : public kinematics::KinematicsBase
{
public:
  /**
   *  @brief Default constructor
   */
  FABRIKKinematicsPlugin();

  bool
  getPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      const std::vector<double>& consistency_limits, std::vector<double>& solution,
      moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      std::vector<double>& solution, const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      const std::vector<double>& consistency_limits, std::vector<double>& solution,
      const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses, const std::vector<double>& ik_seed_state,
                        double timeout, const std::vector<double>& consistency_limits, std::vector<double>& solution,
                        const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions(),
                        const moveit::core::RobotState* context_state = nullptr) const override;

  bool getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                     std::vector<geometry_msgs::Pose>& poses) const override;

  bool initialize(const moveit::core::RobotModel& robot_model, const std::string& group_name,
                  const std::string& base_name, const std::vector<std::string>& tip_frames,
                  double search_discretization) override;

  /**
   * @brief  Return all the joint names in the order they are used internally
   */
  const std::vector<std::string>& getJointNames() const override;

  /**
   * @brief  Return all the link names in the order they are represented internally
   */
  const std::vector<std::string>& getLinkNames() const override;

  /**
   * @brief  Return all the variable names in the order they are represented internally
   */
  const std::vector<std::string>& getVariableNames() const;

protected:
  bool setRedundantJoints(const std::vector<unsigned int>& redundant_joint_indices) override;

private:
  bool timedOut(const ros::WallTime& start_time, double duration) const;

  int getJointIndex(const std::string& name) const;

  bool isRedundantJoint(unsigned int index) const;

  bool active_; /** Internal variable that indicates whether solvers are configured and ready */

  moveit_msgs::KinematicSolverInfo ik_group_info_; /** Stores information for the inverse kinematics solver */

  unsigned int dimension_; /** Dimension of the group */

  const moveit::core::JointModelGroup* joint_model_group_;

  moveit::core::RobotStatePtr robot_state_;

  int num_possible_redundant_joints_;

  std::shared_ptr<ros::ServiceClient> ik_service_client_;
};
}  // namespace srv_kinematics_plugin

