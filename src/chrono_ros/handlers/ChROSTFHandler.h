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
// Authors: Sriram Ashokkumar
// =============================================================================
//
// Handler responsible for publishing information about a ChBody
=======
// Authors: Aaron Young
// =============================================================================
//
// Handler responsible for publishing transform (tf) information
>>>>>>> develop
//
// =============================================================================

#ifndef CH_ROS_TF_HANDLER_H
#define CH_ROS_TF_HANDLER_H

#include "chrono_ros/ChROSHandler.h"
<<<<<<< HEAD
#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"


#include "chrono_ros_interfaces/msg/body.hpp"

#include "chrono/physics/ChBody.h"

=======

#include "chrono/physics/ChBody.h"

#include "tf2_ros/transform_broadcaster.h"

#ifdef CHRONO_PARSERS_URDF
    #include "chrono_parsers/ChParserURDF.h"
#endif
#ifdef CHRONO_SENSOR
    #include "chrono_sensor/sensors/ChSensor.h"
#endif

#include <vector>
#include <utility>
#include <variant>

>>>>>>> develop
namespace chrono {
namespace ros {

/// @addtogroup ros_handlers
/// @{

<<<<<<< HEAD
/// This handler is responsible for publishing state information about a ChBody
class ChROSTFHandler : public ChROSHandler {
  public:
    /// Constructor. body_name is used as the prefix to in the ROS topic.
    ChROSTFHandler(double update_rate, std::shared_ptr<chrono::sensor::ChLidarSensor> lidar, std::shared_ptr<ChBody> body, const std::string& topic_name);

    /// Initializes the handler. Creates a publisher for the body data on the topic "~/output/<body_name>/state"
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    virtual void Tick(double time) override;

  private:
    std::shared_ptr<ChBody> m_body;  ///< The body to publish information about
    std::shared_ptr<chrono::sensor::ChLidarSensor> m_lidar;
    const std::string m_topic_name;          ///< The topic name to publish the body information to
    geometry_msgs::msg::TransformStamped m_msg;  ///< The message to publish
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr
        m_publisher;  ///< The publisher which data is published through
=======
/// @brief This handler is responsible for publishing transform (tf) information. For more information on the use of tf
/// in ROS, see the tf2_ros documentation.
class ChROSTFHandler : public ChROSHandler {
    typedef std::pair<chrono::ChFrame<>, std::string> ChFrameTransform;
    typedef std::variant<std::shared_ptr<chrono::ChBody>, ChFrameTransform> ChROSTransform;

  public:
    /// Constructor.
    ChROSTFHandler(double update_rate);

    /// @brief Initializes the handler. This will create the tf broadcaster. The topic name is assigned to /tf
    /// internally within the broadcaster and this can not be changed.
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

    /// @brief Add a transform to be published. This version of the AddTransform function will use two bodies directly
    /// and calculate the transform at each tick. This is useful for two bodies that are not connected by a link. The
    /// parent and child frame id is queried using the parent->GetName() and child->GetName() methods, respectively.
    /// @param parent The parent body
    /// @param child The child body
    void AddTransform(std::shared_ptr<chrono::ChBody> parent, std::shared_ptr<chrono::ChBody> child);

    /// @brief Add a transform to be published. This version of the AddTransform function will publish a static
    /// transform between the parent and child frame. This is useful for two bodies that are connected by a link. The
    /// parent frame id is queried using the parent->GetName() method.
    /// @param parent The parent body
    /// @param child_frame The child frame
    /// @param child_frame_id The child frame id
    void AddTransform(std::shared_ptr<chrono::ChBody> parent,
                      chrono::ChFrame<double> child_frame,
                      const std::string& child_frame_id);

#ifdef CHRONO_PARSERS_URDF
    /// @brief Add a transform to be published from a URDF file. This method will step through the kinematic tree of the
    /// passed URDF parser and add transforms for each link.
    /// @param parser The URDF parser
    void AddURDF(chrono::parsers::ChParserURDF& parser);
#endif

#ifdef CHRONO_SENSOR
    /// @brief Add a transform to be published from a sensor. This is simply an alias to
    /// AddTransform(sensor->GetParent(), sensor->GetOffsetPose(), frame_id or sensor->GetName()). frame_id will only be
    /// used if set to a non-empty string, otherwise sensor->GetName() will be used.
    /// @param sensor The sensor
    /// @param frame_id The frame id. Defaults to an empty string. If unset, sensor->GetName() will be used.
    void AddSensor(std::shared_ptr<chrono::sensor::ChSensor> sensor, const std::string& frame_id = "");
#endif

  protected:
    /// @brief Update the transforms and publish them
    /// @param time The current time of the simulation
    virtual void Tick(double time) override;

  private:
    std::vector<std::pair<std::shared_ptr<chrono::ChBody>, ChROSTransform>> m_transforms;  ///< The transforms to publish

    std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;  ///< The tf broadcaster
>>>>>>> develop
};

/// @} ros_handlers

}  // namespace ros
}  // namespace chrono

#endif
