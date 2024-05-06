// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young, Luning Bakke
// =============================================================================
//
// Demo showing the integration of ROS with the Rassor rover model
//
// =============================================================================

#include "chrono_models/robot/rassor/Rassor.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono/assets/ChVisualSystem.h"
// #ifdef CHRONO_IRRLICHT
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
// #endif

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/ChROSBodyHandler.h"
#include "chrono_ros/handlers/robot/rassor/ChROSRassorSpeedControlHandler.h"

using namespace chrono;
using namespace chrono::rassor;
using namespace chrono::ros;


// Run-time visualization system (IRRLICHT or VSG)
// ChVisualSystem::Type vis_type = ChVisualSystem::Type::IRRLICHT;

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2023 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl << std::endl;

    // Create the Chrono system with gravity in the negative Z direction
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
    ChCollisionModel::SetDefaultSuggestedMargin(0.0025);

    // Create the ground.
    auto ground_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    auto ground = chrono_types::make_shared<ChBodyEasyBox>(30, 30, 1, 1000, true, true, ground_mat);
    ground->SetPos(ChVector3d(0, 0, -0.5));
    ground->SetFixed(true);
    ground->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/concrete.jpg"), 60, 45);
    sys.Add(ground);

    // Construct a Rassor rover and the asociated driver
    ////auto driver = chrono_types::make_shared<ViperSpeedDriver>(1.0, 5.0);
    auto driver = chrono_types::make_shared<RassorSpeedDriver>(1.0);

    Rassor rassor(&sys, RassorWheelType::RealWheel);
    rassor.SetDriver(driver);
    rassor.Initialize(ChFrame<>(ChVector3d(0, 0, 0.25), QUNIT));
    // driver->SetRazorMotorSpeed((RassorDirID)0, 3.14);
    // driver->SetRazorMotorSpeed((RassorDirID)1, -3.14);

    // Create the run-time visualization interface
// #ifndef CHRONO_IRRLICHT
//     if (vis_type == ChVisualSystem::Type::IRRLICHT)
//         vis_type = ChVisualSystem::Type::VSG;
// #endif

    // std::shared_ptr<ChVisualSystem> vis;
//     switch (vis_type) {
//         case ChVisualSystem::Type::IRRLICHT: {
// #ifdef CHRONO_IRRLICHT
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Viper Rover on Rigid Terrain");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(3, 3, 1));
    vis->AddTypicalLights();
    vis->EnableContactDrawing(ContactsDrawMode::CONTACT_DISTANCES);
    vis->EnableShadows();

//             vis = vis_irr;
// #endif
//             break;
//         }
//         default:
//             throw std::runtime_error("Failed to initialize a visualization method.");
//     }

    // ------------

    // Create ROS manager
    auto ros_manager = chrono_types::make_shared<ChROSManager>();

    // Create a publisher for the simulation clock
    // The clock automatically publishes on every tick and on topic /clock
    auto clock_handler = chrono_types::make_shared<ChROSClockHandler>();
    ros_manager->RegisterHandler(clock_handler);

    // Create a subscriber to the driver inputs
    auto driver_inputs_rate = 25;
    auto driver_inputs_topic_name = "/joint/command";
    auto driver_inputs_handler = chrono_types::make_shared<ChROSRassorSpeedControlHandler>(driver_inputs_rate, driver,
                                                                                            driver_inputs_topic_name);
    ros_manager->RegisterHandler(driver_inputs_handler);

    // Create a publisher for the rover state
    auto rover_state_rate = 25;
    auto rover_state_topic_name = "~/output/rover/state";
    auto rover_state_handler = chrono_types::make_shared<ChROSBodyHandler>(
        rover_state_rate, rassor.GetChassis()->GetBody(), rover_state_topic_name);
    ros_manager->RegisterHandler(rover_state_handler);


    // Create a publisher for the drum
    auto drum_state_rate = 25;
    auto drum_state_topic_name = "~/output/drum_front/state";
    auto drum_state_handler = chrono_types::make_shared<ChROSBodyHandler>(
        drum_state_rate, rassor.GetRazor(RassorDirID::RA_F)->GetBody(), drum_state_topic_name);
    ros_manager->RegisterHandler(drum_state_handler);


    // Finally, initialize the ros manager
    ros_manager->Initialize();

    // ------------

    double time = 0;
    double time_step = 1e-3;
    double time_end = 30;

    // Simulation loop
// #if !defined(CHRONO_IRRLICHT) && !defined(CHRONO_VSG)
    // while (time < time_end) {
// #else
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
// #endif
        // Set current steering angle
        double time = sys.GetChTime();

        std::cout << "Time: " << time << std::endl;

        // std::cout << "position: " << rassor.GetChassisPos() << std::endl;
        // std::cout << "drum: " << rassor.GetRazor((RassorDirID)0)->GetPos() << std::endl;
        // Updates
        rassor.Update();
        if (!ros_manager->Update(time, time_step))
            break;

        sys.DoStepDynamics(time_step);
    }

    return 0;
}