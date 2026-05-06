// Copyright (c) 2026 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka/exception.h>
#include <franka/mobile_model.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

// Ignore warnings from Pinocchio includes
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

#include <Eigen/Core>

#include "urdf_robot_type.h"

namespace franka {

class MobileModel::Data {
 public:
  static constexpr size_t kNumDriveJoints = kNumModules * kJointsPerModule;

  // Upper bound on Pinocchio configuration vector dimension (nq) for any supported mobile robot
  // URDF. Continuous joints use 2 config elements (cos, sin); revolute joints use 1. The TMR
  // URDF has ~15 joints, so nq is well below this limit. Using a fixed-max-size Eigen vector
  // keeps the storage inline and guarantees zero heap allocation in the RT hot path (pose()).
  static constexpr int kMaxConfigDim = 64;
  using ConfigVector = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, kMaxConfigDim, 1>;

  pinocchio::Model model;
  mutable pinocchio::Data data;

  // Pinocchio joint indices for the drive module frames, looked up by name at construction.
  std::array<pinocchio::JointIndex, MobileModel::kNumModules> module_joint_indices{};

  // Mapping from drive joint index to Pinocchio config index for the 4 drive module joints.
  struct JointMapping {
    int idx_q;           // Index into the Pinocchio configuration vector
    bool is_continuous;  // true if nq==2 (cos/sin pair), false if nq==1
  };
  std::array<JointMapping, kNumDriveJoints> joint_mappings{};

  // Pre-allocated configuration buffer (neutral config for passive joints, updated for active).
  // Uses fixed-max-size Eigen vector: storage is inline, zero heap allocation at runtime.
  mutable ConfigVector q_buffer;

  explicit Data(const std::string& urdf_model) {
    if (!isMobileRobotUrdf(urdf_model)) {
      throw ModelException(
          "libfranka: Cannot create MobileModel: the provided URDF does not describe "
          "a mobile robot (robot name must start with 'tmr').");
    }

    pinocchio::urdf::buildModelFromXML(urdf_model, model);
    data = pinocchio::Data(model);

    if (model.nq > kMaxConfigDim) {
      throw ModelException("libfranka: MobileModel: URDF configuration dimension (" +
                           std::to_string(model.nq) + ") exceeds maximum (" +
                           std::to_string(kMaxConfigDim) + ").");
    }

    // Initialize q_buffer with neutral configuration.
    // Revolute joints: 0.0, Continuous joints: (cos(0)=1, sin(0)=0).
    q_buffer = ConfigVector::Zero(model.nq);
    for (int j = 1; j < model.njoints; ++j) {
      if (model.joints[j].nv() == 1 && model.joints[j].nq() == 2) {
        q_buffer[model.joints[j].idx_q()] = 1.0;      // cos(0)
        q_buffer[model.joints[j].idx_q() + 1] = 0.0;  // sin(0)
      }
    }

    // Find the drive module frame joints by name suffix.
    // Convention: front drive module ends with "_joint_1", rear with "_joint_3".
    constexpr std::array<const char*, kNumModules> kModuleSuffixes = {"_joint_1", "_joint_3"};

    for (size_t module_idx = 0; module_idx < kNumModules; ++module_idx) {
      const std::string suffix(kModuleSuffixes[module_idx]);
      bool found = false;
      for (pinocchio::JointIndex j = 1; j < static_cast<pinocchio::JointIndex>(model.njoints);
           ++j) {
        const std::string& name = model.names[j];
        if (name.size() >= suffix.size() &&
            name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0) {
          module_joint_indices[module_idx] = j;
          found = true;
          break;
        }
      }
      if (!found) {
        throw ModelException(
            "libfranka: MobileModel: could not find drive module joint ending with '" + suffix +
            "' in the URDF.");
      }
    }

    // Find and map the 4 drive module joints (steering + drive per module).
    // User array order: {front_steering, front_drive, rear_steering, rear_drive}
    // Convention: suffixes "_joint_0", "_joint_1", "_joint_2", "_joint_3".
    constexpr std::array<const char*, kNumDriveJoints> kJointSuffixes = {"_joint_0", "_joint_1",
                                                                         "_joint_2", "_joint_3"};

    for (size_t i = 0; i < kNumDriveJoints; ++i) {
      const std::string suffix(kJointSuffixes[i]);
      bool found = false;
      for (pinocchio::JointIndex j = 1; j < static_cast<pinocchio::JointIndex>(model.njoints);
           ++j) {
        const std::string& name = model.names[j];
        if (name.size() >= suffix.size() &&
            name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0) {
          joint_mappings[i].idx_q = model.joints[j].idx_q();
          joint_mappings[i].is_continuous =
              (model.joints[j].nq() == 2 && model.joints[j].nv() == 1);
          found = true;
          break;
        }
      }
      if (!found) {
        throw ModelException("libfranka: MobileModel: could not find joint ending with '" + suffix +
                             "' in the URDF.");
      }
    }
  }
};

MobileModel::MobileModel(const std::string& urdf_model)
    : data_(std::make_unique<Data>(urdf_model)) {}

MobileModel::MobileModel(MobileModel&&) noexcept = default;
MobileModel& MobileModel::operator=(MobileModel&&) noexcept = default;
MobileModel::~MobileModel() noexcept = default;

std::array<double, 16> MobileModel::pose(MobileFrame frame,
                                         const MobileJointPositions& joint_positions) const {
  // Update only the 4 drive module joints in the pre-allocated buffer. Zero heap allocation.
  for (size_t i = 0; i < kNumModules * kJointsPerModule; ++i) {
    const auto& mapping = data_->joint_mappings[i];
    if (mapping.is_continuous) {
      data_->q_buffer[mapping.idx_q] = std::cos(joint_positions.drive_modules[i]);
      data_->q_buffer[mapping.idx_q + 1] = std::sin(joint_positions.drive_modules[i]);
    } else {
      data_->q_buffer[mapping.idx_q] = joint_positions.drive_modules[i];
    }
  }

  pinocchio::forwardKinematics(data_->model, data_->data, data_->q_buffer);

  // Map MobileFrame to Pinocchio joint index via the pre-computed lookup table
  size_t module_index = 0;
  switch (frame) {
    case MobileFrame::kFrontDriveModule:
      module_index = 0;
      break;
    case MobileFrame::kRearDriveModule:
      module_index = 1;
      break;
    default:
      throw std::invalid_argument("libfranka: Invalid MobileFrame given.");
  }

  pinocchio::JointIndex joint_index = data_->module_joint_indices[module_index];

  std::array<double, 16> result;
  Eigen::Map<Eigen::Matrix4d>(result.data()) = data_->data.oMi[joint_index].toHomogeneousMatrix();
  return result;
}

}  // namespace franka
