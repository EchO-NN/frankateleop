// Copyright (c) 2026 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <franka/exception.h>
#include <franka/mobile_model.h>

#include "test_utils.h"

class GivenMobileModel : public ::testing::Test {
 protected:
  std::string mobile_urdf;
  std::string arm_urdf;

  GivenMobileModel()
      : mobile_urdf(
            franka_test_utils::readFileToString(franka_test_utils::getMobileUrdfPath(__FILE__))),
        arm_urdf(franka_test_utils::readFileToString(franka_test_utils::getArmUrdfPath(__FILE__))) {
  }
};

TEST_F(GivenMobileModel, WhenConstructedFromMobileUrdf_ThenSucceeds) {
  ASSERT_NO_THROW(franka::MobileModel model(mobile_urdf));
}

TEST_F(GivenMobileModel, WhenConstructedFromArmUrdf_ThenThrows) {
  ASSERT_THROW(franka::MobileModel model(arm_urdf), franka::ModelException);
}

TEST_F(GivenMobileModel, WhenPoseAtZeroConfig_ThenReturnsValidHomogeneousTransform) {
  franka::MobileModel model(mobile_urdf);

  franka::MobileJointPositions joint_positions{};

  auto pose1 = model.pose(franka::MobileFrame::kFrontDriveModule, joint_positions);
  auto pose2 = model.pose(franka::MobileFrame::kRearDriveModule, joint_positions);

  // Both should be valid 4x4 homogeneous matrices
  // Check that the last row is [0 0 0 1] (column-major)
  // In column-major: element [3,0]=pose[3], [3,1]=pose[7], [3,2]=pose[11], [3,3]=pose[15]
  ASSERT_NEAR(pose1[3], 0.0, franka_test_utils::kEps);
  ASSERT_NEAR(pose1[7], 0.0, franka_test_utils::kEps);
  ASSERT_NEAR(pose1[11], 0.0, franka_test_utils::kEps);
  ASSERT_NEAR(pose1[15], 1.0, franka_test_utils::kEps);

  ASSERT_NEAR(pose2[3], 0.0, franka_test_utils::kEps);
  ASSERT_NEAR(pose2[7], 0.0, franka_test_utils::kEps);
  ASSERT_NEAR(pose2[11], 0.0, franka_test_utils::kEps);
  ASSERT_NEAR(pose2[15], 1.0, franka_test_utils::kEps);
}

TEST_F(GivenMobileModel, WhenPoseComputed_ThenRotationPartIsOrthogonal) {
  franka::MobileModel model(mobile_urdf);
  franka::MobileJointPositions joint_positions{};

  auto pose = model.pose(franka::MobileFrame::kFrontDriveModule, joint_positions);

  // Extract 3x3 rotation (column-major: columns are pose[0..2], pose[4..6], pose[8..10])
  Eigen::Matrix3d R;
  R << pose[0], pose[4], pose[8], pose[1], pose[5], pose[9], pose[2], pose[6], pose[10];

  // R^T * R should be identity
  Eigen::Matrix3d RtR = R.transpose() * R;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      double expected = (i == j) ? 1.0 : 0.0;
      ASSERT_NEAR(RtR(i, j), expected, franka_test_utils::kEps)
          << "R^T*R not identity at (" << i << "," << j << ")";
    }
  }
}

TEST_F(GivenMobileModel, WhenDifferentFramesQueried_ThenPosesDiffer) {
  franka::MobileModel model(mobile_urdf);
  franka::MobileJointPositions joint_positions{};

  auto pose1 = model.pose(franka::MobileFrame::kFrontDriveModule, joint_positions);
  auto pose2 = model.pose(franka::MobileFrame::kRearDriveModule, joint_positions);

  // The two drive modules are at different positions, so poses should differ
  bool any_different = false;
  for (size_t i = 0; i < 16; ++i) {
    if (std::abs(pose1[i] - pose2[i]) > franka_test_utils::kEps) {
      any_different = true;
      break;
    }
  }
  ASSERT_TRUE(any_different) << "Front and rear drive module poses should be different";
}

TEST_F(GivenMobileModel, WhenJointConfigurationChanges_ThenPoseChanges) {
  franka::MobileModel model(mobile_urdf);

  franka::MobileJointPositions q_zero{};
  auto pose_zero = model.pose(franka::MobileFrame::kFrontDriveModule, q_zero);

  // Change front steering angle
  franka::MobileJointPositions q_changed{};
  q_changed.drive_modules[0] = 0.5;  // Rotate front drive module steering
  auto pose_changed = model.pose(franka::MobileFrame::kFrontDriveModule, q_changed);

  // The pose should have changed
  bool any_different = false;
  for (size_t i = 0; i < 16; ++i) {
    if (std::abs(pose_zero[i] - pose_changed[i]) > franka_test_utils::kEps) {
      any_different = true;
      break;
    }
  }
  ASSERT_TRUE(any_different) << "Pose should change with different joint configuration";
}
