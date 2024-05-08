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

// #include "chrono_ros_interfaces/msg/rassor_wheel_id.hpp"
// #include "chrono_ros_interfaces/msg/rassor_drum_id.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

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

    m_subscription = node->create_subscription<sensor_msgs::msg::JointState>(
        m_topic_name, 50, std::bind(&ChROSRassorSpeedControlHandler::Callback, this, _1));

    return true;
}

void ChROSRassorSpeedControlHandler::Callback(const sensor_msgs::msg::JointState& msg) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_msg = msg;

    messageQueue.push(msg);

//     std::cout << "(Callback) Received JointState message:\n";
//     for (size_t i = 0; i < msg.name.size(); ++i) {
//         std::cout << "Joint: " << msg.name[i] << ", Position: " << msg.position[i];
//         std::cout << std::endl;
// }




}

void ChROSRassorSpeedControlHandler::Tick(double time) {
    std::lock_guard<std::mutex> lock(m_mutex);
    // for (size_t i = 0; i < m_msg.name.size(); ++i){
        
        std::cout << "Tick: " << std::endl;
        while (!messageQueue.empty()){

            auto msg = messageQueue.front();
             messageQueue.pop();


            if (msg.name.size() != 0 ){

            std::string name = msg.name[0];
            float vel = msg.velocity[0];
            float angle = msg.position[0];

            std::cout << name << ", " << vel << ", " << angle << std::endl;


            if (name == "LF/driving") {
                m_driver->SetDriveMotorSpeed((RassorWheelID)0, vel);
            } else if (name == "LR/driving") {
                m_driver->SetDriveMotorSpeed((RassorWheelID)2, vel);
            } else if (name == "RF/driving") {
                m_driver->SetDriveMotorSpeed((RassorWheelID)1, vel);
            } else if (name == "RR/driving") {
                m_driver->SetDriveMotorSpeed((RassorWheelID)3, vel);
            } else if (name == "FB/shoulder") {
                m_driver->SetArmMotorAngle((RassorDirID)0, -angle);  // front bucket 
            } else if (name == "RB/shoulder") {
                m_driver->SetArmMotorAngle((RassorDirID)1, +angle);   // rear bucket 
            } else if (name == "FB/bucket") {
                m_driver->SetRazorMotorSpeed((RassorDirID)0, -vel);
            } else if (name == "RB/bucket") {
                m_driver->SetRazorMotorSpeed((RassorDirID)1, vel);
            }

        }

    }

}

}  // namespace ros
}  // namespace chrono
