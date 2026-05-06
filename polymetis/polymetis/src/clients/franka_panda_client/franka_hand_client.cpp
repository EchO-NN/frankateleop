#include "polymetis/clients/franka_hand_client.hpp"

#include <atomic>
#include <cmath>
#include <limits>
#include "spdlog/spdlog.h"
#include <string>
#include <thread>
#include <time.h>

#include <grpc/grpc.h>

#include "polymetis.grpc.pb.h"
#include "polymetis/utils.h"

using grpc::ClientContext;

FrankaHandClient::FrankaHandClient(std::shared_ptr<grpc::Channel> channel,
                                   YAML::Node config)
    : stub_(GripperServer::NewStub(channel)) {
  // Connect to gripper
  std::string robot_ip = config["robot_ip"].as<std::string>();
  spdlog::info("Connecting to robot_ip {}", robot_ip);
  gripper_.reset(new franka::Gripper(robot_ip));

  // Initialize gripper
  gripper_->homing();
  is_moving_.store(false);
  stop_requested_.store(false);

  servo_mode_ = config["servo_mode"] && config["servo_mode"].as<bool>();
  command_deadband_ =
      config["command_deadband"]
          ? config["command_deadband"].as<double>()
          : (servo_mode_ ? 0.002 : 0.045);
  spdlog::info("Franka hand servo_mode={}, command_deadband={} m",
               servo_mode_, command_deadband_);

  // Initialize server connection
  franka::GripperState franka_gripper_state = gripper_->readOnce();

  GripperMetadata metadata;
  metadata.set_max_width(franka_gripper_state.max_width);
  metadata.set_hz(GRIPPER_HZ);

  ClientContext context;
  Empty empty;
  stub_->InitRobotClient(&context, metadata, &empty);

  spdlog::info("Connected.", robot_ip);
}

void FrankaHandClient::getGripperState(void) {
  franka::GripperState franka_gripper_state = gripper_->readOnce();

  gripper_state_.set_width(franka_gripper_state.width);
  gripper_state_.set_is_grasped(franka_gripper_state.is_grasped);
  gripper_state_.set_is_moving(is_moving_.load());
  gripper_state_.set_prev_command_successful(prev_cmd_successful_.load());

  // gripper_state.time();  // Use current timestamp instead!
  setTimestampToNow(gripper_state_.mutable_timestamp());
}

void FrankaHandClient::applyGripperCommand(GripperCommand gripper_cmd) {
  is_moving_.store(true);
  stop_requested_.store(false);

  try {
    if (gripper_cmd.grasp()) {
      spdlog::info("Grasping at width {} at speed={}", gripper_cmd.width(),
                   gripper_cmd.speed());
      double eps_inner = 0.1;
      double eps_outer = 0.1;
      prev_cmd_successful_.store(
          gripper_->grasp(gripper_cmd.width(), gripper_cmd.speed(),
                          gripper_cmd.force(), eps_inner, eps_outer));

    } else {
      spdlog::info("Moving to width {} at speed={}", gripper_cmd.width(),
                   gripper_cmd.speed());
      prev_cmd_successful_.store(
          gripper_->move(gripper_cmd.width(), gripper_cmd.speed()));
    }
  } catch (const std::exception &e) {
    if (stop_requested_.load()) {
      spdlog::debug("Active gripper command was preempted: {}", e.what());
      prev_cmd_successful_.store(true);
    } else {
      spdlog::warn("Gripper command failed: {}", e.what());
      prev_cmd_successful_.store(false);
    }
  }

  is_moving_.store(false);
  stop_requested_.store(false);
}

void FrankaHandClient::run(void) {
  const long period_ns = static_cast<long>(1.0e9 / GRIPPER_HZ);

  int timestamp_ns = 0;
  float cmd_width = 0.0;
  float prev_cmd_width = std::numeric_limits<float>::quiet_NaN();

  struct timespec abs_target_time;
  clock_gettime(CLOCK_REALTIME, &abs_target_time);
  while (true) {
    // Run control step
    try {
      getGripperState();
    } catch (const std::exception &e) {
      spdlog::warn("Failed to read gripper state: {}", e.what());
      prev_cmd_successful_.store(false);
    }

    grpc::ClientContext context;
    status_ = stub_->ControlUpdate(&context, gripper_state_, &gripper_cmd_);

    // Skip if command not updated.
    timestamp_ns = gripper_cmd_.timestamp().nanos();
    cmd_width = gripper_cmd_.width();
    bool is_new_command =
        timestamp_ns && timestamp_ns != prev_cmd_timestamp_ns_;
    bool width_changed = std::isnan(prev_cmd_width) ||
                         std::fabs(cmd_width - prev_cmd_width) >
                             command_deadband_;

    if (is_new_command && width_changed) {
      if (is_moving_.load()) {
        if (servo_mode_ && !stop_requested_.exchange(true)) {
          spdlog::debug("Preempting active gripper move for servo target {}",
                        cmd_width);
          try {
            gripper_->stop();
          } catch (const std::exception &e) {
            spdlog::warn("Failed to stop active gripper move: {}", e.what());
          }
        }
      } else {
        GripperCommand command = gripper_cmd_;
        std::thread th(&FrankaHandClient::applyGripperCommand, this, command);
        th.detach();
        prev_cmd_timestamp_ns_ = timestamp_ns;
        prev_cmd_width = cmd_width;
      }
    }

    // Spin once
    abs_target_time.tv_nsec += period_ns;
    while (abs_target_time.tv_nsec >= 1000000000L) {
      abs_target_time.tv_nsec -= 1000000000L;
      abs_target_time.tv_sec += 1;
    }
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &abs_target_time, nullptr);
  }
}

int main(int argc, char *argv[]) {
  if (argc != 2) {
    spdlog::error("Usage: franka_hand_client /path/to/cfg.yaml");
    return 1;
  }
  YAML::Node config = YAML::LoadFile(argv[1]);

  // Launch client
  std::string control_address = config["control_ip"].as<std::string>() + ":" +
                                config["control_port"].as<std::string>();
  FrankaHandClient franka_hand_client(
      grpc::CreateChannel(control_address, grpc::InsecureChannelCredentials()),
      config);
  franka_hand_client.run();

  // Termination
  spdlog::info("Wait for shutdown; press CTRL+C to close.");

  return 0;
}
