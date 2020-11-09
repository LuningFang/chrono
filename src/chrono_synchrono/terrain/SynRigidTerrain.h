// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
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
// Class that wraps and synchronizes rigid terrain between Chrono Systems
// See chrono_vehicle/terrain/RigidTerrain for the physics
//
// =============================================================================

#ifndef SYN_RIGID_TERRAIN_H
#define SYN_RIGID_TERRAIN_H

#include "chrono_synchrono/terrain/SynTerrain.h"

#include "chrono_vehicle/terrain/RigidTerrain.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_terrain
/// @{

class SYN_API SynRigidTerrain : public SynTerrain {
  public:
    // Default Constructor
    SynRigidTerrain(std::shared_ptr<RigidTerrain> terrain = nullptr) { SetTerrain(terrain); }

    // Construct the underlying rigid terrain from the specified JSON file
    SynRigidTerrain(ChSystem* system, const std::string& filename);

    // Destructor
    ~SynRigidTerrain() {}

    /// @brief Processes incoming message - nothing to be synced for rigid terrain
    virtual void ProcessMessage(SynMessage* message) override {}

    /// @brief Generate outgoing message - nothing to be synced for rigid terrain
    virtual void GenerateMessagesToSend(std::vector<SynMessage*>& messages, int rank) override {}

    /// Set the terrain
    void SetTerrain(std::shared_ptr<RigidTerrain> terrain) { m_rigid_terrain = terrain; }

    /// Get the terrain
    virtual std::shared_ptr<ChTerrain> GetTerrain() override { return m_rigid_terrain; }
    std::shared_ptr<RigidTerrain> GetRigidTerrain() { return m_rigid_terrain; }

  private:
    void AddVisualizationAssetsJSON(const rapidjson::Value& a);

    std::shared_ptr<RigidTerrain> m_rigid_terrain;  ///< Underlying RigidTerrain object that is synchronized
};

/// @} synchrono_terrain

}  // namespace synchrono
}  // namespace chrono

#endif