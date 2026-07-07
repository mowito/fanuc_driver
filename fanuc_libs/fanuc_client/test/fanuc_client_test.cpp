// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mutex>
#include <thread>
#include <vector>

#include "fanuc_client/fanuc_client.hpp"

class MockStreamMotionConnection : public stream_motion::StreamMotionInterface
{
public:
  // status_packet_delay_ms paces getStatusPacket() to emulate the real-time cadence of a real
  // network-backed connection. Defaults to 0 (free-spinning, as before) so existing tests that
  // don't care about real-time pacing are unaffected; tests that need the realtime thread's
  // virtual clock to track wall-clock time (e.g. to reproduce queue-starvation timing) pass a
  // non-zero value matching the control period.
  explicit MockStreamMotionConnection(std::atomic<bool>& stream_connected, int status_packet_delay_ms = 0)
    : stream_connected_{ stream_connected }, status_packet_delay_ms_{ status_packet_delay_ms }
  {
    status_.status = 15;
    status_.joint_angle = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  }

  bool streamConnected() const
  {
    return stream_connected_;
  }

  void sendStartPacket() const override
  {
    stream_connected_ = true;
  }

  void sendStopPacket() const override
  {
    stream_connected_ = false;
  }

  void sendCommand(const std::array<double, stream_motion::kMaxAxisNumber>& command_pos, bool is_last_command,
                   const std::array<uint8_t, 256>& io_command) const override
  {
    for (int i = 0; i < stream_motion::kMaxAxisNumber; ++i)
    {
      status_.joint_angle[i] = static_cast<float>(command_pos[i]);
    }
    std::lock_guard<std::mutex> lock(history_mutex_);
    command_history_.push_back(command_pos);
  }

  // Returns every command_pos ever passed to sendCommand, in order. Thread-safe: sendCommand is
  // called from the FanucClient realtime thread while this is typically read from the test thread.
  std::vector<std::array<double, stream_motion::kMaxAxisNumber>> commandHistory() const
  {
    std::lock_guard<std::mutex> lock(history_mutex_);
    return command_history_;
  }

  bool getStatusPacket(stream_motion::RobotStatusPacket& status) override
  {
    if (!stream_connected_)
    {
      return false;
    }
    if (status_packet_delay_ms_ > 0)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(status_packet_delay_ms_));
    }
    status = status_;
    return true;
  }

  bool getRobotLimits(const uint32_t axis_number, stream_motion::RobotThresholdPacket& robot_threshold_velocity,
                      stream_motion::RobotThresholdPacket& robot_threshold_acceleration,
                      stream_motion::RobotThresholdPacket& robot_threshold_jerk) const override
  {
    robot_threshold_velocity.axis_number = axis_number;
    robot_threshold_acceleration.axis_number = axis_number;
    robot_threshold_jerk.axis_number = axis_number;
    for (int i = 0; i < 20; ++i)
    {
      robot_threshold_velocity.full_payload[i] = 200.0f;
      robot_threshold_velocity.no_payload[i] = 200.0f;
      robot_threshold_acceleration.full_payload[i] = 2000.0f;
      robot_threshold_acceleration.no_payload[i] = 2000.0f;
      robot_threshold_jerk.full_payload[i] = 20000.0f;
      robot_threshold_jerk.no_payload[i] = 20000.0f;
    }
    return true;
  }

  bool configureGPIO(const stream_motion::GPIOConfiguration& config) const override
  {
    return true;
  }

  bool getControllerCapability(stream_motion::ControllerCapabilityResultPacket& controller_capability) override
  {
    controller_capability.sampling_rate = 8;
    return true;
  }

  void configureForceSensor(uint32_t do_reset, uint32_t force_sensor_type) const override
  {
  }

private:
  mutable stream_motion::RobotStatusPacket status_;
  std::atomic<bool>& stream_connected_;
  int status_packet_delay_ms_;
  mutable std::mutex history_mutex_;
  mutable std::vector<std::array<double, stream_motion::kMaxAxisNumber>> command_history_;
};

class MockRMIConnection : public rmi::RMIConnectionInterface
{
public:
  ~MockRMIConnection() override = default;

  MOCK_METHOD(rmi::ConnectROS2Packet::Response, connect, (std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::DisconnectPacket::Response, disconnect, (std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::InitializePacket::Response, initializeRemoteMotion, (std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::ProgramCallPacket::Response, programCall,
              (const std::string& program_name, std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::ProgramCallPacket::Request, programCallNonBlocking, (const std::string& program_name), (override));
  MOCK_METHOD(rmi::StatusRequestPacket::Response, getStatus, (std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::SetSpeedOverridePacket::Response, setSpeedOverride, (int value, std::optional<double> timeout),
              (override));
  MOCK_METHOD(rmi::AbortPacket::Response, abort, (std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::PausePacket::Response, pause, (std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::ContinuePacket::Response, resume, (std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::ResetRobotPacket::Response, reset, (std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::ReadErrorPacket::Response, readError, (std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::WritePositionRegisterPacket::Response, writePositionRegister,
              (int register_number, const std::string& representation, const rmi::ConfigurationData& configuration,
               const rmi::PositionData& position, const rmi::JointAngleData& joint_angle, std::optional<double> timeout),
              (override));
  MOCK_METHOD(rmi::ReadPositionRegisterPacket::Response, readPositionRegister,
              (int register_number, std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::ReadNumericRegisterPacket::Response, readNumericRegister,
              (int register_number, std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::WriteNumericRegisterPacket::Response, writeNumericRegister,
              (int register_number, (std::variant<int, float>)value, std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::ReadDigitalInputPortPacket::Response, readDigitalInputPort,
              (uint16_t port_number, std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::WriteDigitalOutputPacket::Response, writeDigitalOutputPort,
              (uint16_t port_number, bool port_value, std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::ReadIOPortPacket::Response, readIOPort,
              (const std::string& port_type, int port_number, std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::WriteIOPortPacket::Response, writeIOPort,
              (int port_number, const std::string& port_type, (std::variant<int, float>)port_value,
               std::optional<double> timeout),
              (override));
  rmi::ReadVariablePacket::Response readVariablePacket(const std::string& variable_name,
                                                       std::optional<double> timeout) override
  {
    assert(variable_name == std::string("$STMO.$COM_INT"));
    rmi::ReadVariablePacket::Response response{};
    response.VariableValue = 8;
    return response;
  }
  MOCK_METHOD(rmi::WriteVariablePacket::Response, writeVariablePacket,
              (const std::string& variable_name, (std::variant<int, float>)value, std::optional<double> timeout),
              (override));
  MOCK_METHOD(rmi::GetExtendedStatusPacket::Response, getExtendedStatus, (std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::SetPayloadPacket::Response, setPayloadSchedule,
              (uint8_t payload_schedule_number, std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::SetPayloadValuePacket::Response, setPayloadValue,
              (uint8_t payload_schedule_number, float mass, float cg_x, float cg_y, float cg_z, bool use_in, float in_x,
               float in_y, float in_z, std::optional<double> timeout),
              (override));
  MOCK_METHOD(rmi::SetPayloadCompPacket::Response, setPayloadComp,
              (uint8_t payload_schedule_number, float mass, float cg_x, float cg_y, float cg_z, float in_x, float in_y,
               float in_z, std::optional<double> timeout),
              (override));
  MOCK_METHOD(rmi::JointMotionJRepPacket::Response, sendJointMotion,
              (rmi::JointMotionJRepPacket::Request joint_motion_request, const std::optional<double> timeout),
              (override));
  MOCK_METHOD(rmi::ReadJointAnglesPacket::Response, readJointAngles,
              (const std::optional<uint8_t>& group, const std::optional<double> timeout), (override));
  MOCK_METHOD(std::optional<rmi::SystemFaultPacket>, checkSystemFault, (), (override));
  MOCK_METHOD(std::optional<rmi::TimeoutTerminatePacket>, checkTimeoutTerminate, (), (override));
  MOCK_METHOD(std::optional<rmi::CommunicationPacket>, checkCommunicationPacket, (), (override));
  MOCK_METHOD(std::optional<rmi::UnknownPacket>, checkUnknownPacket, (), (override));
};

using NiceMockStreamMotionConnection = testing::NiceMock<MockStreamMotionConnection>;
using NiceMockRMIConnection = testing::NiceMock<MockRMIConnection>;

TEST(FanucClientTest, TestSuccessfulLifecycle)
{
  std::atomic<bool> stream_connected = false;
  auto stream_motion_interface = std::make_unique<NiceMockStreamMotionConnection>(stream_connected);
  auto rmi_interface = std::make_unique<NiceMockRMIConnection>();
  fanuc_client::FanucClient fanuc_client("127.0.0.1", 60015, 16001, std::move(stream_motion_interface),
                                         std::move(rmi_interface));

  // Reading/Writing data before starting the stream should throw an error
  const Eigen::VectorXd initial_joint_targets = Eigen::VectorXd::Zero(stream_motion::kMaxAxisNumber);
  EXPECT_THROW(fanuc_client.writeJointTarget(initial_joint_targets), std::runtime_error);
  EXPECT_THROW(fanuc_client.readJointAngles(), std::runtime_error);

  // Start the real-time stream with a mock connection
  fanuc_client.startRealtimeStream();
  while (!stream_connected)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(stream_connected);

  // After starting the stream, we the first read should return the initial joint angles
  Eigen::VectorXd joint_states = fanuc_client.readJointAngles();
  EXPECT_EQ(joint_states, initial_joint_targets);

  // Writing joint targets should update the joint states eventually
  Eigen::VectorXd joint_targets = joint_states.array() + 1.0;
  while (joint_states == initial_joint_targets)
  {
    fanuc_client.writeJointTarget(joint_targets);
    joint_states = fanuc_client.readJointAngles();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_EQ(joint_targets, joint_states);

  // Stop the real-time stream should disconnect the stream
  fanuc_client.stopRealtimeStream();
  while (stream_connected)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_FALSE(stream_connected);
}

// Regression test for a bug where FanucClient::streamMotionThread's queue bracket-tracking loop
// could snap directly to a not-yet-due future waypoint in a single control cycle instead of
// ramping smoothly toward it, whenever the command queue starved for longer than one control
// period (e.g. after a scheduling/network hiccup upstream) and then received a new command. See
// command_pos.csv field data: a ~144ms flat hold followed by a single-cycle ~17 degree jump.
TEST(FanucClientTest, NoSnapAfterQueueStarvation)
{
  std::atomic<bool> stream_connected = false;
  // Pace the mocked realtime loop to the control period (8ms, matching getControllerCapability's
  // sampling_rate below) so the interpolator's virtual clock tracks wall-clock time closely enough
  // to reproduce the same starvation timing seen in the field.
  constexpr int kControlPeriodMs = 8;
  auto stream_motion_interface = std::make_unique<NiceMockStreamMotionConnection>(stream_connected, kControlPeriodMs);
  NiceMockStreamMotionConnection* mock_stream_motion = stream_motion_interface.get();
  auto rmi_interface = std::make_unique<NiceMockRMIConnection>();
  fanuc_client::FanucClient fanuc_client("127.0.0.1", 60015, 16001, std::move(stream_motion_interface),
                                         std::move(rmi_interface));

  fanuc_client.startRealtimeStream();
  while (!stream_connected)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  const Eigen::VectorXd steady_target = Eigen::VectorXd::Zero(stream_motion::kMaxAxisNumber);
  for (int i = 0; i < 20; ++i)
  {
    fanuc_client.writeJointTarget(steady_target);
    std::this_thread::sleep_for(std::chrono::milliseconds(kControlPeriodMs));
  }

  // Starve the command queue for well over one control period, matching the field-observed
  // ~144ms gap that preceded the jump in command_pos.csv. The realtime thread keeps running
  // during this gap (graceful hold at steady_target), it just receives no new commands.
  std::this_thread::sleep_for(std::chrono::milliseconds(150));

  const Eigen::VectorXd distant_target = steady_target.array() + 1.0;
  for (int i = 0; i < 40; ++i)
  {
    fanuc_client.writeJointTarget(distant_target);
    std::this_thread::sleep_for(std::chrono::milliseconds(kControlPeriodMs));
  }

  fanuc_client.stopRealtimeStream();

  const auto history = mock_stream_motion->commandHistory();
  ASSERT_GT(history.size(), 2u);

  constexpr double kNear = 1e-3;
  size_t first_near_distant = history.size();
  for (size_t i = 0; i < history.size(); ++i)
  {
    if (std::abs(history[i][0] - distant_target[0]) < kNear)
    {
      first_near_distant = i;
      break;
    }
  }
  ASSERT_LT(first_near_distant, history.size()) << "distant_target was never reached";

  size_t last_near_steady = 0;
  for (size_t i = 0; i < first_near_distant; ++i)
  {
    if (std::abs(history[i][0] - steady_target[0]) < kNear)
    {
      last_near_steady = i;
    }
  }

  // Correct behavior ramps gradually across many cycles (~150ms starvation / 8ms period is
  // roughly 19 cycles); the bug snaps from steady_target straight to distant_target in a single
  // cycle right after the starvation gap ends. Require more than a handful of cycles in between
  // to catch a single-cycle (or near single-cycle) snap while tolerating normal test timing jitter.
  EXPECT_GT(first_near_distant - last_near_steady, 3u)
      << "Position went from steady_target to distant_target in " << (first_near_distant - last_near_steady)
      << " cycle(s) — expected a gradual multi-cycle ramp; this indicates an interpolation snap.";
}

TEST(FanucClientTest, TestGetLimits)
{
  std::atomic<bool> stream_connected = false;
  auto stream_motion_interface = std::make_unique<MockStreamMotionConnection>(stream_connected);
  auto rmi_interface = std::make_unique<NiceMockRMIConnection>();
  fanuc_client::FanucClient fanuc_client("127.0.0.1", 60015, 16001, std::move(stream_motion_interface),
                                         std::move(rmi_interface));
  const double v_peak = 1000.0;
  const double payload = 0.0;
  std::vector<double> vel_limit;
  std::vector<double> acc_limit;
  std::vector<double> jerk_limit;
  fanuc_client.getLimits(v_peak, payload, vel_limit, acc_limit, jerk_limit);
  for (int i = 0; i < stream_motion::kMaxAxisNumber; ++i)
  {
    EXPECT_EQ(vel_limit[i], 200.0f);
    EXPECT_EQ(acc_limit[i], 2000.0f);
    EXPECT_EQ(jerk_limit[i], 20000.0f);
  }
}

TEST(RMISington, OnlyOneInstanceCreated)
{
  // Create the first MockRMIConnection instance and set it as the singleton
  auto rmi_inst = std::make_unique<MockRMIConnection>();
  fanuc_client::RMISingleton::setRMIInstance(std::move(rmi_inst));

  // Retrieve the singleton instance twice and check that both references point to the same object
  auto rmi_connection_inst1 = fanuc_client::RMISingleton::getRMIInstance();
  auto rmi_connection_inst2 = fanuc_client::RMISingleton::getRMIInstance();
  EXPECT_EQ(rmi_connection_inst1.get(), rmi_connection_inst2.get());

  // Create a new MockRMIConnection instance and replace the singleton
  auto new_rmi_inst = std::make_unique<MockRMIConnection>();
  MockRMIConnection* new_rmi_inst_ptr = new_rmi_inst.get();
  fanuc_client::RMISingleton::setRMIInstance(std::move(new_rmi_inst));

  // Retrieve the singleton instance again and check that it has changed to the new object
  auto rmi_connection_inst3 = fanuc_client::RMISingleton::getRMIInstance();
  EXPECT_NE(rmi_connection_inst2.get(), rmi_connection_inst3.get());
  EXPECT_EQ(rmi_connection_inst3.get(), new_rmi_inst_ptr);
}
