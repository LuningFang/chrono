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
// Handler responsible for publishing information about a ChBody
//
// =============================================================================

#include "chrono_ros/handlers/ChROSTFHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
// #include "tf2_msgs/msgtf_message.hpp"
// #include "geometry_msgs/msg/transform_stamped.hpp"
#include "chrono/core/ChTransform.h"


namespace chrono {
namespace ros {

ChROSTFHandler::ChROSTFHandler(double update_rate, std::shared_ptr<chrono::sensor::ChLidarSensor> lidar, std::shared_ptr<ChBody> body, const std::string& topic_name)
    : ChROSHandler(update_rate), m_lidar(lidar), m_body(body), m_topic_name(topic_name) {}

bool ChROSTFHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    auto node = interface->GetNode();

    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic_name)) {
        return false;
    }

    m_publisher = node->create_publisher<tf2_msgs::msg::TFMessage>(m_topic_name, 1);

    m_msg.header.frame_id = "lidar"; //TODO

    return true;
}

void ChROSTFHandler::Tick(double time) {
    auto cobra_pos = m_body->GetPos();
    auto lidar_pos = m_lidar->GetOffsetPose().GetPos();

    // auto rot = m_body->GetRot();
    // auto lin_vel = m_body->GetPos_dt();
    // auto ang_vel = m_body->GetWvel_loc();
    // auto lin_acc = m_body->GetPos_dtdt();
    // auto ang_acc = m_body->GetWacc_loc();

    m_msg.header.stamp = ChROSHandlerUtilities::GetROSTimestamp(time);

    // m_msg.transform.translation.x = cobra_pos[0] + lidar_pos[0];
    // m_msg.transform.translation.y = cobra_pos[1] + lidar_pos[1];
    // m_msg.transform.translation.z = cobra_pos[2] + lidar_pos[2];

    // m_msg.transform.rotation.x = 0;
    // m_msg.transform.rotation.y = 0;
    // m_msg.transform.rotation.z = 0;
    // m_msg.transform.rotation.w = 1;

    // auto frame = TransformLocalToParent(m_body->GetFrame_REF_to_abs(), m_lidar->GetOffsetPose());
    // auto lidar_frame = m_lidar->GetOffsetPose();
    // auto output = lidar_frame.TransformLocalToParent(ChVector<>(0, 0, 1));
    // auto output = ChFrame<>(0, 0, 1).TransformLocalToParent(lidar_frame.GetPos());

    // auto output = chrono::ChFrame<>::TransformLocalToParent(lidar_frame, ChFrame<>(0, 0, 1));
    // auto output_pos = output.GetPos();
    // auto output_pos = lidar_frame.GetRot();
    // m_msg.transform.translation.x = output.x();
    // m_msg.transform.translation.y = output.y();
    // m_msg.transform.translation.z = output.z();

    auto lidar_frame = m_lidar->GetOffsetPose();
    ChFrame<> output_frame;
    ChFrame<> lidarTemp = m_lidar->GetOffsetPose();
    ChFrame<> bodyTemp = m_body->GetFrame_COG_to_abs();
    bodyTemp.TransformLocalToParent(lidarTemp, output_frame);

    m_msg.transform.translation.x = output_frame.GetPos().x();
    m_msg.transform.translation.y = output_frame.GetPos().y();
    m_msg.transform.translation.z = output_frame.GetPos().z();


    // flip one of these

    m_msg.transform.rotation.x = output_frame.GetRot().e0();
    m_msg.transform.rotation.y = output_frame.GetRot().e1();
    m_msg.transform.rotation.z = output_frame.GetRot().e2();
    m_msg.transform.rotation.w = output_frame.GetRot().e3();





    m_msg.child_frame_id = "base_link";


    // m_msg.transform.rotation.x = 0;

    tf2_msgs::msg::TFMessage msg;
    msg.transforms.push_back(m_msg);




    m_publisher->publish(msg);
}

}  // namespace ros
}  // namespace chrono
