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
<<<<<<< HEAD
// Authors: Sriram Ashokkumar, Json Zhou
// =============================================================================
//
// Handler responsible for publishing information about a tf - currently just set up for lidar
=======
// Authors: Aaron Young
// =============================================================================
//
// Handler responsible for publishing transform (tf) information
>>>>>>> develop
//
// =============================================================================

#include "chrono_ros/handlers/ChROSTFHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
<<<<<<< HEAD
// #include "tf2_msgs/msgtf_message.hpp"
// #include "geometry_msgs/msg/transform_stamped.hpp"
#include "chrono/core/ChTransform.h"

=======

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
>>>>>>> develop

namespace chrono {
namespace ros {

<<<<<<< HEAD
ChROSTFHandler::ChROSTFHandler(double update_rate, std::shared_ptr<chrono::sensor::ChLidarSensor> lidar, std::shared_ptr<ChBody> body, const std::string& topic_name)
    : ChROSHandler(update_rate), m_lidar(lidar), m_body(body), m_topic_name(topic_name) {}
=======
ChROSTFHandler::ChROSTFHandler(double update_rate) : ChROSHandler(update_rate) {}
>>>>>>> develop

bool ChROSTFHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    auto node = interface->GetNode();

<<<<<<< HEAD
    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic_name)) {
        return false;
    }

    m_publisher = node->create_publisher<tf2_msgs::msg::TFMessage>(m_topic_name, 1);

    m_msg.header.frame_id = "base_link"; //TODO
    m_msg.child_frame_id = "lidar";
        
=======
    m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);
>>>>>>> develop

    return true;
}

<<<<<<< HEAD
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

    m_msg.transform.translation.x = 1;
    m_msg.transform.translation.y = 0;
    m_msg.transform.translation.z = 1;

    ChQuaternion<> quat = Q_from_AngAxis(0, ChVector<>(1, 0, 0));
    m_msg.transform.rotation.x = quat.e1();
    m_msg.transform.rotation.y = quat.e2();
    m_msg.transform.rotation.z = quat.e3();
    m_msg.transform.rotation.w = quat.e0();
    //std::cout << quat.e1() << "," << quat.e2() << "," << quat.e3() << "," << quat.e0() << std::endl;
    // m_msg.transform.rotation.x = output_frame.GetRot().e2();
    // m_msg.transform.rotation.y = output_frame.GetRot().e1();
    // m_msg.transform.rotation.z = output_frame.GetRot().e0();
    // m_msg.transform.rotation.w = output_frame.GetRot().e3();


    // flip one of these

    // // Extract the original rotation from `output_frame`
    // ChQuaternion<> original_rotation = output_frame.GetRot();

    // // Create a quaternion for a 180-degree rotation around the Z-axis (for example)
    // ChQuaternion<> flip_rotation = Q_from_AngAxis(chrono::CH_C_PI, ChVector<>(0, 0, 1));

    // // Apply the flip rotation
    // ChQuaternion<> new_rotation = original_rotation * flip_rotation;

    // m_msg.transform.rotation.x = new_rotation.e0();
    // m_msg.transform.rotation.y = new_rotation.e1();
    // m_msg.transform.rotation.z = new_rotation.e2();
    // m_msg.transform.rotation.w = new_rotation.e3();





    m_msg.child_frame_id = "lidar";
    


    // m_msg.transform.rotation.x = 0;

    tf2_msgs::msg::TFMessage msg;
    msg.transforms.push_back(m_msg);




    m_publisher->publish(msg);
=======
void ChROSTFHandler::AddTransform(std::shared_ptr<chrono::ChBody> parent, std::shared_ptr<chrono::ChBody> child) {
    if (!parent) {
        std::cerr << "ChROSTFHandler::AddTransform: Parent body is null" << std::endl;
        return;
    } else if (!child) {
        std::cerr << "ChROSTFHandler::AddTransform: Child body is null" << std::endl;
        return;
    }

    m_transforms.push_back(std::make_pair(parent, child));
}

void ChROSTFHandler::AddTransform(std::shared_ptr<chrono::ChBody> parent,
                                  chrono::ChFrame<double> child_frame,
                                  const std::string& child_frame_id) {
    if (!parent) {
        std::cerr << "ChROSTFHandler::AddTransform: Parent body is null" << std::endl;
        return;
    }

    ChFrameTransform child = std::make_pair(child_frame, child_frame_id);
    m_transforms.push_back(std::make_pair(parent, child));
}

#ifdef CHRONO_PARSERS_URDF
void ChROSTFHandler::AddURDF(chrono::parsers::ChParserURDF& parser) {
    auto model = parser.GetModelTree();

    std::vector<urdf::LinkSharedPtr> links;
    model->getLinks(links);

    for (const auto& link : links) {
        auto joint = link->parent_joint;
        if (!joint)
            continue;

        auto parent = parser.GetChBody(joint->parent_link_name);
        auto child = parser.GetChBody(joint->child_link_name);
        AddTransform(parent, child);
    }
}
#endif

#ifdef CHRONO_SENSOR
void ChROSTFHandler::AddSensor(std::shared_ptr<chrono::sensor::ChSensor> sensor, const std::string& frame_id) {
    if (!sensor) {
        std::cerr << "ChROSTFHandler::AddSensor: Sensor is null" << std::endl;
        return;
    }

    auto parent = sensor->GetParent();
    auto child_frame = sensor->GetOffsetPose();
    auto child_frame_id = frame_id.empty() ? sensor->GetName() : frame_id;
    AddTransform(parent, child_frame, child_frame_id);
}
#endif

geometry_msgs::msg::TransformStamped CreateTransformStamped(chrono::ChFrame<> local_to_parent,
                                                            const std::string& parent_frame_id,
                                                            const std::string& child_frame_id,
                                                            double time) {
    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = ChROSHandlerUtilities::GetROSTimestamp(time);
    tf_msg.header.frame_id = parent_frame_id;
    tf_msg.child_frame_id = child_frame_id;

    auto pos = local_to_parent.GetPos();
    tf_msg.transform.translation.x = pos.x();
    tf_msg.transform.translation.y = pos.y();
    tf_msg.transform.translation.z = pos.z();

    auto quat = local_to_parent.GetRot();
    tf_msg.transform.rotation.w = quat.e0();
    tf_msg.transform.rotation.x = quat.e1();
    tf_msg.transform.rotation.y = quat.e2();
    tf_msg.transform.rotation.z = quat.e3();

    return tf_msg;
}

void ChROSTFHandler::Tick(double time) {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    for (auto const& [parent, child] : m_transforms) {
        chrono::ChFrame<> child_to_parent;
        std::string child_frame_id;
        if (std::holds_alternative<std::shared_ptr<chrono::ChBody>>(child)) {
            auto child_body = std::get<std::shared_ptr<chrono::ChBody>>(child);
            child_to_parent = parent->GetFrameRefToAbs().GetInverse() * child_body->GetFrameRefToAbs();
            child_frame_id = child_body->GetName();
        } else {
            auto frame_pair = std::get<ChFrameTransform>(child);
            child_to_parent = frame_pair.first;
            child_frame_id = frame_pair.second;
        }

        auto tf_msg = CreateTransformStamped(child_to_parent, parent->GetName(), child_frame_id, time);
        transforms.push_back(tf_msg);
    }

    m_tf_broadcaster->sendTransform(transforms);
>>>>>>> develop
}

}  // namespace ros
}  // namespace chrono
