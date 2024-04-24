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

#include "chrono_ros/handlers/robot/rassor/ChROSRassorSpeedControlHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"

#include "chrono_ros_interfaces/msg/rassor_wheel_id.hpp"
#include "chrono_ros_interfaces/msg/rassor_drum_id.hpp"

using std::placeholders::_1;

using namespace chrono::rassor;

namespace chrono {
namespace ros {

ChROSRassorSpeedControlHandler::ChROSRassorSpeedControlHandler(double update_rate,
                                                               std::shared_ptr<RassorSpeedDriver> driver,
                                                               const std::string& topic_name)
    : ChROSHandler(update_rate), m_driver(driver), m_topic_name(topic_name) {}

bool ChROSRassorSpeedControlHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    auto node = interface->GetNode();

    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic_name)) {
        return false;
    }

    m_subscription = node->create_subscription<chrono_ros_interfaces::msg::RassorDriver>(
        m_topic_name, 1, std::bind(&ChROSRassorSpeedControlHandler::Callback, this, _1));

    return true;
}

void ChROSRassorSpeedControlHandler::Callback(const chrono_ros_interfaces::msg::RassorDriver& msg) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_msg = msg;
}

void ChROSRassorSpeedControlHandler::Tick(double time) {
    std::lock_guard<std::mutex> lock(m_mutex);

    // This is the real stuff 
    // for (auto steering_command : m_msg.driver_commands.steering_list) {
    //     if (steering_command.wheel_id != chrono_ros_interfaces::msg::ViperWheelID::V_UNDEFINED)
    //         m_driver->SetSteering(steering_command.angle,
    //                               static_cast<chrono::viper::ViperWheelID>(steering_command.wheel_id));
    //     else
    //         m_driver->SetSteering(steering_command.angle);
    // }

    // m_driver->SetLifting(m_msg.driver_commands.lifting);
    // m_driver->SetMotorStallTorque(m_msg.stall_torque.torque,
    //                               static_cast<chrono::viper::ViperWheelID>(m_msg.stall_torque.wheel_id));
    // m_driver->SetMotorNoLoadSpeed(m_msg.no_load_speed.speed,
    //                               static_cast<chrono::viper::ViperWheelID>(m_msg.no_load_speed.wheel_id));
}

}  // namespace ros
}  // namespace chrono
