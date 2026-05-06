// Copyright (c) 2026 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <string>

namespace franka {

/**
 * Determines whether the given URDF describes a mobile robot.
 *
 * A URDF is considered a mobile robot if the robot name attribute starts with "tmr".
 * For example: \<robot name="tmrv0_2"\> is a mobile robot.
 *
 * @param[in] urdf The URDF XML string.
 * @return true if the URDF describes a mobile robot, false otherwise.
 */
bool isMobileRobotUrdf(const std::string& urdf);

}  // namespace franka
