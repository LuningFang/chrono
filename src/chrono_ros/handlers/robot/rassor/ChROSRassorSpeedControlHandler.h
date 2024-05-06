// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young
// =============================================================================
//
// Handler responsible for receiving and updating the DC motor control commands
// for the Viper rover
//
// =============================================================================

#ifndef CHROSRASSORSPEEDCONTROLHANDLER_H
#define CHROSRASSORSPEEDCONTROLHANDLER_H

#include "chrono_ros/ChROSHandler.h"

#include "chrono_models/robot/rassor/Rassor.h"

#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <mutex>
#include <queue>

namespace chrono {
namespace ros {

/// @addtogroup ros_robot_handlers
/// @{

/// This handler is responsible for interfacing a ViperDCMotorControl driver to ROS. Will instantiate a subscriber to
/// chrono_ros_interfaces::msg::ViperDCMotorControl.
class ChROSRassorSpeedControlHandler : public ChROSHandler {
  public:
    /// Constructor. Takes a ViperDCMotorControl driver
    ChROSRassorSpeedControlHandler(double update_rate,
                                    std::shared_ptr<chrono::rassor::RassorSpeedDriver> driver,
                                    const std::string& topic_name);

    /// Initializes the handler.
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    /// Updates the driver with stored inputs data from Callback
    virtual void Tick(double time) override;

  private:
    /// NOTE: This will only update the local m_inputs variable. The driver will receive
    /// the new commands in the Tick() function.
    void Callback(const sensor_msgs::msg::JointState& msg);

  private:
    std::shared_ptr<chrono::rassor::RassorSpeedDriver> m_driver;  ///< handle to the driver

    const std::string m_topic_name;                         ///< name of the topic to publish to
    sensor_msgs::msg::JointState m_msg;  ///< message to publish
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        m_subscription;  ///< the publisher for the imu message

    std::mutex m_mutex;

    std::queue<sensor_msgs::msg::JointState> messageQueue;
};

/// @} ros_robot_handlers

}  // namespace ros
}  // namespace chrono

#endif
