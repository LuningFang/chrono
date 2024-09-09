// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Front chassis subsystem for the articulated vehicle.
//
// =============================================================================

#include "subsystems/ACV_ChassisFront.h"

using namespace chrono;
using namespace chrono::vehicle;

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double ACV_ChassisFront::m_body_mass = 2000;
const ChVector3d ACV_ChassisFront::m_body_inertiaXX(100, 400, 500);
const ChVector3d ACV_ChassisFront::m_body_inertiaXY(0, 0, 0);
const ChVector3d ACV_ChassisFront::m_body_COM_loc(0, 0, 0.4);
const ChVector3d ACV_ChassisFront::m_connector_loc(-1.0, 0, 0.1);
const ChCoordsys<> ACV_ChassisFront::m_driverCsys(ChVector3d(0.0, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ACV_ChassisFront::ACV_ChassisFront(const std::string& name, bool fixed) : ChRigidChassis(name, fixed) {
    m_body_inertia(0, 0) = m_body_inertiaXX.x();
    m_body_inertia(1, 1) = m_body_inertiaXX.y();
    m_body_inertia(2, 2) = m_body_inertiaXX.z();

    m_body_inertia(0, 1) = m_body_inertiaXY.x();
    m_body_inertia(0, 2) = m_body_inertiaXY.y();
    m_body_inertia(1, 2) = m_body_inertiaXY.z();
    m_body_inertia(1, 0) = m_body_inertiaXY.x();
    m_body_inertia(2, 0) = m_body_inertiaXY.y();
    m_body_inertia(2, 1) = m_body_inertiaXY.z();

    // Visualization primitives
    utils::ChBodyGeometry::BoxShape box(ChVector3d(-0.25, 0.0, 0.1), ChQuaternion<>(1, 0, 0, 0), ChVector3d(1.5, 1.0, 0.2));
    utils::ChBodyGeometry::CylinderShape cyl(ChVector3d(0.5, 0, 0), ChVector3d(0, 1, 0), 0.05, 2);

    m_geometry.vis_boxes.push_back(box);
    m_geometry.vis_cylinders.push_back(cyl);

    m_geometry.color_boxes = ChColor(0.2f, 0.2f, 0.4f);
    m_geometry.color_cylinders = ChColor(0.2f, 0.2f, 0.4f);
}
