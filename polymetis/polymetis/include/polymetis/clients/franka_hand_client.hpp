#include "spdlog/spdlog.h"
#include "yaml-cpp/yaml.h"

#include "polymetis/utils.h"
#include <atomic>
#include <grpcpp/grpcpp.h>

#include <franka/gripper.h>
#include <franka/gripper_state.h>

#define GRIPPER_HZ 30

// Define tolerances to be able to grasp any object without specifying width
#define EPSILON_INNER 0.2
#define EPSILON_OUTER 0.2

class FrankaHandClient {
private:
  void getGripperState(void);
  void applyGripperCommand(GripperCommand gripper_cmd);

  // gRPC
  std::unique_ptr<GripperServer::Stub> stub_;
  grpc::Status status_;

  GripperState gripper_state_;
  GripperCommand gripper_cmd_;
  int prev_cmd_timestamp_ns_ = 0;
  std::atomic<bool> prev_cmd_successful_{true};
  bool servo_mode_ = false;
  double command_deadband_ = 0.045;

  // Franka
  std::shared_ptr<franka::Gripper> gripper_;
  std::atomic<bool> is_moving_{false};
  std::atomic<bool> stop_requested_{false};

public:
  FrankaHandClient(std::shared_ptr<grpc::Channel> channel, YAML::Node config);
  void run(void);
};
