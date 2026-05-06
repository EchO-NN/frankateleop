// Copyright (c) 2026 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <gtest/gtest.h>

#include "test_utils.h"
#include "urdf_robot_type.h"

TEST(UrdfRobotType, GivenFr3Urdf_WhenChecked_ThenIsNotMobile) {
  std::string urdf =
      franka_test_utils::readFileToString(franka_test_utils::getArmUrdfPath(__FILE__));
  ASSERT_FALSE(urdf.empty());
  ASSERT_FALSE(franka::isMobileRobotUrdf(urdf));
}

TEST(UrdfRobotType, GivenTmrUrdf_WhenChecked_ThenIsMobile) {
  std::string urdf =
      franka_test_utils::readFileToString(franka_test_utils::getMobileUrdfPath(__FILE__));
  ASSERT_FALSE(urdf.empty());
  ASSERT_TRUE(franka::isMobileRobotUrdf(urdf));
}

TEST(UrdfRobotType, GivenRobotNameStartingWithTmr_WhenChecked_ThenIsMobile) {
  std::string urdf = R"(<?xml version="1.0" ?>
<robot name="tmrv0_2">
  <link name="base"/>
</robot>)";
  ASSERT_TRUE(franka::isMobileRobotUrdf(urdf));
}

TEST(UrdfRobotType, GivenRobotNameExactlyTmr_WhenChecked_ThenIsMobile) {
  std::string urdf = R"(<?xml version="1.0" ?>
<robot name="tmr">
  <link name="base"/>
</robot>)";
  ASSERT_TRUE(franka::isMobileRobotUrdf(urdf));
}

TEST(UrdfRobotType, GivenArmRobotName_WhenChecked_ThenIsNotMobile) {
  std::string urdf = R"(<?xml version="1.0" ?>
<robot name="fr3">
  <link name="base"/>
</robot>)";
  ASSERT_FALSE(franka::isMobileRobotUrdf(urdf));
}

TEST(UrdfRobotType, GivenRobotNameContainingTmrButNotAsPrefix_WhenChecked_ThenIsNotMobile) {
  std::string urdf = R"(<?xml version="1.0" ?>
<robot name="my_tmr_robot">
  <link name="base"/>
</robot>)";
  ASSERT_FALSE(franka::isMobileRobotUrdf(urdf));
}

TEST(UrdfRobotType, GivenNoRobotNameAttribute_WhenChecked_ThenReturnsFalse) {
  std::string urdf = R"(<?xml version="1.0" ?>
<robot>
  <link name="base"/>
</robot>)";
  ASSERT_FALSE(franka::isMobileRobotUrdf(urdf));
}

TEST(UrdfRobotType, GivenInvalidXml_WhenChecked_ThenReturnsFalse) {
  ASSERT_FALSE(franka::isMobileRobotUrdf("not valid xml at all"));
}

TEST(UrdfRobotType, GivenEmptyString_WhenChecked_ThenReturnsFalse) {
  ASSERT_FALSE(franka::isMobileRobotUrdf(""));
}

TEST(UrdfRobotType, GivenNoRobotElement_WhenChecked_ThenReturnsFalse) {
  std::string urdf = R"(<?xml version="1.0" ?><something_else/>)";
  ASSERT_FALSE(franka::isMobileRobotUrdf(urdf));
}
