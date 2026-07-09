#pragma once

#include <cstddef>

#include <moveit/robot_model/robot_model.h>
#include <moveit/online_signal_smoothing/smoothing_base_class.h>
#include <fanuc_moveit_config/moveit_ruckig_filter_parameters.hpp>

#include <ruckig/ruckig.hpp>

namespace online_signal_smoothing
{

class RuckigFilterPlugin : public SmoothingBaseClass
{
public:
  /**
   * Initialize the smoothing algorithm
   * @param node ROS node, used for parameter retrieval
   * @param robot_model used to retrieve vel/accel/jerk limits
   * @param num_joints number of actuated joints in the JointGroup
   * @return True if initialization was successful
   */
  bool initialize(rclcpp::Node::SharedPtr node, moveit::core::RobotModelConstPtr robot_model,
                  size_t num_joints) override;

  /**
   * Smooth the command signals for all DOF
   * @param position_vector array of joint position commands
   * @return True if initialization was successful
   */
  bool doSmoothing(std::vector<double>& position_vector) override;

  /**
   * Reset to a given joint state
   * @param joint_positions reset the filters to these joint positions
   * @return True if reset was successful
   */
  bool reset(const std::vector<double>& joint_positions) override;

private:
  /**
   * A utility to print Ruckig's internal state
   */
  void printRuckigState();

  /**
   * A utility to get velocity/acceleration/jerk bounds from the robot model
   * @return true if all bounds are defined
   */
  bool getVelAccelJerkBounds(std::vector<double>& joint_velocity_bounds, std::vector<double>& joint_acceleration_bounds,
                             std::vector<double>& joint_jerk_bounds);

  /** \brief Parameters loaded from yaml file at runtime */
  online_signal_smoothing::Params params_;
  /** \brief The robot model contains the vel/accel/jerk limits that Ruckig requires */
  moveit::core::RobotModelConstPtr robot_model_;
  bool have_initial_ruckig_output_ = false;
  std::optional<ruckig::Ruckig<ruckig::DynamicDOFs>> ruckig_;
  std::optional<ruckig::InputParameter<ruckig::DynamicDOFs>> ruckig_input_;
  std::optional<ruckig::OutputParameter<ruckig::DynamicDOFs>> ruckig_output_;
};
}  // namespace online_signal_smoothing