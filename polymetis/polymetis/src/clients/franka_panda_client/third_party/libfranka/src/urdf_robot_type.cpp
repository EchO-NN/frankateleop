// Copyright (c) 2026 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "urdf_robot_type.h"

#include <cstring>

#include <tinyxml2.h>

namespace franka {

constexpr const char* kMobileRobotPrefix = "tmr";

bool isMobileRobotUrdf(const std::string& urdf) {
  tinyxml2::XMLDocument doc;
  if (doc.Parse(urdf.c_str()) != tinyxml2::XML_SUCCESS) {
    return false;
  }

  const tinyxml2::XMLElement* robot = doc.FirstChildElement("robot");
  if (robot == nullptr) {
    return false;
  }

  const char* name_attr = robot->Attribute("name");
  if (name_attr == nullptr) {
    return false;
  }

  return std::strncmp(name_attr, kMobileRobotPrefix, std::strlen(kMobileRobotPrefix)) == 0;
}

}  // namespace franka
