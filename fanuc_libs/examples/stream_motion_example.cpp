// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <cmath>
#include <iostream>
#include <fstream>

#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "stream_motion/stream.hpp"

// Number of revolute joints on the arm (J1-J6). The stream motion command/status
// arrays are sized for stream_motion::kMaxAxisNumber to also support extended axes,
// but only the first kNumArmJoints entries participate in the Jacobian IK below.
constexpr int kNumArmJoints = 6;

// Builds the kinematic chain used to convert a Cartesian twist of the tool frame into
// joint velocities. Returns false if the URDF can't be parsed or the chain can't be
// extracted between base_link and tool_link.
bool loadArmChain(const std::string& urdf_path, const std::string& base_link, const std::string& tool_link,
                   KDL::Chain& chain)
{
  KDL::Tree tree;
  if (!kdl_parser::treeFromFile(urdf_path, tree))
  {
    std::cerr << "Failed to parse URDF from " << urdf_path << std::endl;
    return false;
  }
  if (!tree.getChain(base_link, tool_link, chain))
  {
    std::cerr << "Failed to extract chain from " << base_link << " to " << tool_link << std::endl;
    return false;
  }
  return true;
}

void printStatus(const stream_motion::RobotStatusPacket& status)
{
  std::cout << "packet_type: " << status.packet_type << std::endl;
  std::cout << "version_no: " << status.version_no << std::endl;
  std::cout << "sequence_no: " << status.sequence_no << std::endl;
  std::cout << "status: " << static_cast<int>(status.status) << std::endl;
  std::cout << "robot_status: " << static_cast<int>(status.robot_status) << std::endl;
  std::cout << "contact_stop_status: " << static_cast<int>(status.contact_stop_status) << std::endl;
  std::cout << "time_stamp: " << status.time_stamp << std::endl;
  for (int i = 0; i < status.position.size(); ++i)
  {
    std::cout << "position[" << i << "]: " << status.position[i] << std::endl;
  }
  for (int i = 0; i < status.joint_angle.size(); ++i)
  {
    std::cout << "joint_angle[" << i << "]: " << status.joint_angle[i] << std::endl;
  }
  for (int i = 0; i < status.current.size(); ++i)
  {
    std::cout << "current[" << i << "]: " << status.current[i] << std::endl;
  }
  std::cout << "status.io_status[1]: " << (int)status.io_status[1] << std::endl;
}

void printJointLimits(const stream_motion::RobotThresholdPacket& robot_threshold_velocity,
                      const stream_motion::RobotThresholdPacket& robot_threshold_acceleration,
                      const stream_motion::RobotThresholdPacket& robot_threshold_jerk)
{
  std::cout << "Velocity limits:" << std::endl;
  for (size_t i = 0; i < 20; ++i)
  {
    std::cout << "  joint[" << i << "]: " << robot_threshold_velocity.no_payload[i] << std::endl;
  }
  std::cout << "Acceleration limits:" << std::endl;
  for (size_t i = 0; i < 20; ++i)
  {
    std::cout << "  joint[" << i << "]: " << robot_threshold_acceleration.no_payload[i] << std::endl;
  }
  std::cout << "Jerk limits:" << std::endl;
  for (size_t i = 0; i < 20; ++i)
  {
    std::cout << "  joint[" << i << "]: " << robot_threshold_jerk.no_payload[i] << std::endl;
  }
}
int main()
{
  // Crete IO config for reading and writing
  std::array<uint8_t, 256> io_command{};
  for (int i = 0; i < 256; ++i)
  {
    io_command[i] = 0xFF;
  }

  std::array<stream_motion::GPIOControlConfig, 32> gpio_config{};
  gpio_config[0].command_type = stream_motion::GPIOCommandType::IOCmd;
  gpio_config[0].gpio_type = static_cast<uint32_t>(stream_motion::IOType::F);
  gpio_config[0].start = 1;
  gpio_config[0].length = 32;

  gpio_config[1].command_type = stream_motion::GPIOCommandType::IOState;
  gpio_config[1].gpio_type = static_cast<uint32_t>(stream_motion::IOType::F);
  gpio_config[1].start = 1;
  gpio_config[1].length = 32;

  std::ofstream command_pos_log("command_pos.csv", std::ios::out | std::ios::trunc);
  command_pos_log << "timestamp";
  for (int i = 0; i < stream_motion::kMaxAxisNumber; ++i)
  {
    command_pos_log << ",joint_" << i;
  }
  command_pos_log << "\n";

  stream_motion::StreamMotionConnection connection("172.168.1.9");
  stream_motion::RobotThresholdPacket robot_threshold_velocity;
  stream_motion::RobotThresholdPacket robot_threshold_acceleration;
  stream_motion::RobotThresholdPacket robot_threshold_jerk;

  // Get the robot limits for axis 1 and print them out
  connection.getRobotLimits(1, robot_threshold_velocity, robot_threshold_acceleration, robot_threshold_jerk);
  printJointLimits(robot_threshold_velocity, robot_threshold_acceleration, robot_threshold_jerk);

  // Setup GPIO
  connection.configureGPIO(gpio_config);

  // Start the streaming protocol
  connection.sendStartPacket();
  stream_motion::RobotStatusPacket status{};
  connection.getStatusPacket(status);

  // Print the initial status packet
  printStatus(status);

  std::array<double, stream_motion::kMaxAxisNumber> initial_command{};
  std::transform(status.joint_angle.begin(), status.joint_angle.end(), initial_command.begin(),
                 [](const float angle) { return static_cast<double>(angle); });
  std::array<double, stream_motion::kMaxAxisNumber> current_command = initial_command;

  // Build the kinematic chain (CRX-10iA/L, expanded from
  // fanuc_crx_description/urdf/crx10ia_l_urdf_macro.xacro) used to convert the hardcoded
  // tool twist below into joint velocities via the Jacobian pseudo-inverse.
  KDL::Chain arm_chain;
  if (!loadArmChain("/home/mowito/fanuc_ws/src/crx10ia_l.urdf", "base_link", "flange", arm_chain) ||
      static_cast<int>(arm_chain.getNrOfJoints()) != kNumArmJoints)
  {
    std::cerr << "Failed to build a " << kNumArmJoints << "-DOF arm chain for Jacobian-based IK" << std::endl;
    return 1;
  }
  KDL::ChainIkSolverVel_pinv ik_vel_solver(arm_chain);

  // Hardcoded Cartesian twist of the tool frame, expressed in the base frame:
  // [vx, vy, vz, wx, wy, wz] in m/s and rad/s.
  
  const KDL::Twist tool_twist(KDL::Vector(0.02, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.0));
  // Drive the robot along the hardcoded twist for 5 seconds
  constexpr int period = 5000;
  constexpr int num_steps = period / 8;
  const double dt = 0.008;
  for (int i = 0; i < num_steps; ++i)
  {
    // If we fail to get the status packet due to timeout, we need to send the stop packet to clean up the connection
    if (!connection.getStatusPacket(status))
    {
      break;
    }

    // command_pos is in degrees; KDL expects radians.
    KDL::JntArray q_current(kNumArmJoints);
    for (int j = 0; j < kNumArmJoints; ++j)
    {
      q_current(j) = current_command[j] * M_PI / 180.0;
    }

    // Resolve the tool twist into joint velocities: qdot = J(q)^+ * twist
    KDL::JntArray qdot(kNumArmJoints);
    const int ik_result = ik_vel_solver.CartToJnt(q_current, tool_twist, qdot);
    if (ik_result < 0)
    {
      std::cerr << "Velocity IK failed with error code " << ik_result << std::endl;
      break;
    }

    // Integrate the joint velocities (rad/s) into the next position command (degrees).
    for (int j = 0; j < kNumArmJoints; ++j)
    {
      current_command[j] += qdot(j) * (180.0 / M_PI) * dt;
    }

    // Send the command to the robot
    connection.sendCommand(current_command, false, io_command);
    // Log the command position
    command_pos_log << i * dt;
    for (int j = 0; j < stream_motion::kMaxAxisNumber; ++j)
    {
      command_pos_log << "," << current_command[j];
    }
    command_pos_log << "\n";  
  }

  command_pos_log.close();

  if (connection.getStatusPacket(status))
  {
    connection.sendCommand(initial_command, true, io_command);
  }

  // checking motion completed
  bool wait_for_completed = true;
  while (wait_for_completed)
  {
    connection.getStatusPacket(status);
    wait_for_completed = status.status & 1 || status.status & 8;
  }

  // Terminate the streaming protocol
  connection.sendStopPacket();

  // Print the final status packet
  printStatus(status);
}
