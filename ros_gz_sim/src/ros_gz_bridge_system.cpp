// Copyright 2024 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <gz/common/Util.hh>
#include <gz/plugin/Register.hh>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_bridge/bridge_config.hpp>
#include <ros_gz_bridge/ros_gz_bridge.hpp>

#include "ros_gz_bridge_system.hpp"

namespace ros_gz_sim
{
/// \brief Private ROSGzBridgeSystem data class.
class ROSGzBridgeSystemPrivate
{
public:
  /// \brief The ROS 2 <--> Gz bridge.
  std::shared_ptr<ros_gz_bridge::RosGzBridge> bridge;

  /// \brief The ROS 2 executor.
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec;

  /// \brief A thread to call spin and not block the Gazebo thread.
  std::thread thread;
};

//////////////////////////////////////////////////
ROSGzBridgeSystem::ROSGzBridgeSystem()
: System(), dataPtr(new ROSGzBridgeSystemPrivate())
{
}

//////////////////////////////////////////////////
ROSGzBridgeSystem::~ROSGzBridgeSystem()
{
  if (this->dataPtr->exec) {
    this->dataPtr->exec->cancel();
    this->dataPtr->thread.join();
  }
}

//////////////////////////////////////////////////
void ROSGzBridgeSystem::Configure(
  const gz::sim::Entity & /*_entity*/,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & /*_ecm*/,
  gz::sim::EventManager & /*_eventMgr*/)
{
  // Ensure that ROS is setup.
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  if (!_sdf->HasElement("config_file")) {
    std::cerr << "No <config_file> found. Plugin disabled." << std::endl;
    return;
  }

  // Sanity check: Make sure that the config file exists and it's a file.
  std::string filename = _sdf->Get<std::string>("config_file");
  std::string path = gz::common::findFile(filename);
  if (!gz::common::isFile(path)) {
    std::cerr << "Unable to open YAML file [" << filename
              << "], check your GZ_SIM_RESOURCE_PATH settings." << std::endl;
    return;
  }

  // Create the bridge passing the parameters as rclcpp::NodeOptions().
  this->dataPtr->bridge = std::make_shared<ros_gz_bridge::RosGzBridge>(
    rclcpp::NodeOptions().append_parameter_override("config_file", path));

  // Create the executor.
  this->dataPtr->exec =
    std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  this->dataPtr->exec->add_node(this->dataPtr->bridge);

  // Spin in a separate thread to not block Gazebo.
  this->dataPtr->thread = std::thread([this]() {this->dataPtr->exec->spin();});
}
}  // namespace ros_gz_sim

GZ_ADD_PLUGIN(
  ros_gz_sim::ROSGzBridgeSystem,
  gz::sim::System,
  ros_gz_sim::ROSGzBridgeSystem::ISystemConfigure)

GZ_ADD_PLUGIN_ALIAS(
  ros_gz_sim::ROSGzBridgeSystem,
  "ros_gz_sim::ROSGzBridge")
