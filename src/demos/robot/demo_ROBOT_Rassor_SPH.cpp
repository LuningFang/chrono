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
// Author: Json Zhou, Luning Bakke
// Chrono::FSI demo to show usage of Rassor rover models on CRM granular terrain
// This demo uses a plug-in Rassor rover model from chrono::models
// TODO: 
// rover initialization and driver should be independent from create solid phase
// =============================================================================

#include "chrono_models/robot/rassor/Rassor.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/visualization/ChFsiVisualization.h"
#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_VSG
    #include "chrono_fsi/visualization/ChFsiVisualizationVSG.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::rassor;


// If true, save as Wavefront OBJ; if false, save as VTK
bool save_obj = false;

// Physical properties of terrain particles
double iniSpacing = 0.004;
double kernelLength = 0.004;
double density = 2600.0;

// Dimension of the space domain
double bxDim = 3.0; // this for real
//double bxDim = 3.0; // use this for debug
double byDim = 0.9;
double bzDim = 0.1;

// Rover initial location
ChVector3d init_loc(-bxDim / 2.0 + 1.0, 0, bzDim + 0.25);

// Simulation time and stepsize
double total_time = 10.0;
double dT = 2.0e-4;

// Save data as csv files to see the results off-line using Paraview
bool output = true;
int out_fps = 10;

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = true;
float render_fps = 100;

// Pointer to store the Rassor instance
std::shared_ptr<Rassor> rover;
std::shared_ptr<RassorSpeedDriver> driver;

// Define Viper rover wheel type
RassorWheelType wheel_type = RassorWheelType::RealWheel;

// Use below mesh file if the wheel type is real VIPER wheel
std::string wheel_obj = "robot/rassor/obj/rassor_wheel.obj";

// Speeds from Sam
double rover_velocity_array[5] = {0.05, 0.1, 0.15, 0.2, 0.3};
double bucket_omega_array[5]   = {0.7, 1.39, 2.09, 2.78, 4.17};


std::shared_ptr<ChContactMaterial> CustomWheelMaterial(ChContactMethod contact_method);
// Forward declaration of helper functions
void CreateSolidPhase(ChSystemNSC& sysMBS, ChSystemFsi& sysFSI);
std::vector<ChVector3d> LoadSolidPhaseBCE(std::string filename);
bool CreateSubDirectories(std::string out_dir);

int main(int argc, char* argv[]) {

    // check number of command line inputs
    //if (argc != 4) {
    //    std::cout << "usage: ./demo_ROBOT_Rassor_SPH <TestID> <artificial_viscosity> <output_folder>" << std::endl;
    //    return 1;
    //}

    // get the TestID from the command line
    int TestID = 2;
    double artificial_viscosity = 0.1;
    std::string out_dir = "full_vehicle_raise_drum";

    double wheel_radius = 0.22;
    double wheel_driver_speed  = rover_velocity_array[TestID] / wheel_radius;
    double bucket_driver_speed = bucket_omega_array[TestID];

    // Output directories and settings
    out_dir = GetChronoOutputPath() + out_dir + "/";

    if (!CreateSubDirectories(out_dir)) { return 1; }

    // create a csv file that writes down all the output
    std::ofstream info_file;
    info_file.open(out_dir + "/info.csv");
    // write down header line
    // time,x,y,z,yaw,pitch,roll,front_fx,front_fy,front_fz,front_tx,front_ty,front_tz,back_fx,back_fy,back_fz,back_tx,back_ty,back_tz
    info_file << "time,x,y,z,roll,pitch,yaw,front_fx,front_fy,front_fz,front_tx,front_ty,front_tz,back_fx,back_fy,back_"
                 "fz,back_tx,back_ty,back_tz"
              << std::endl;


    // Create a physical system and a corresponding FSI system
    ChSystemNSC sysMBS;
    ChSystemFsi sysFSI(&sysMBS);

    ChVector3d gravity = ChVector3d(0, 0, -9.81);
    sysMBS.SetGravitationalAcceleration(gravity);
    sysFSI.SetGravitationalAcceleration(gravity);

    // Read JSON file with simulation parameters
    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_Rassor_granular_NSC.json");
    sysFSI.ReadParametersFromFile(inputJson);

    // overwrite artificial viscosity 
    sysFSI.SetArtificialViscosity(artificial_viscosity, 0.0);

    // Set the initial particle spacing
    sysFSI.SetInitialSpacing(iniSpacing);

    std::cout << "kernelL" << kernelLength << std::endl;

    // Set the SPH kernel length
    sysFSI.SetKernelLength(kernelLength);

    // Set the terrain density
    sysFSI.SetDensity(density);

    // Set the simulation stepsize
    sysFSI.SetStepSize(dT);

    // Set the simulation domain size
    sysFSI.SetContainerDim(ChVector3d(bxDim, byDim, bzDim));

    // Set SPH discretization type, consistent or inconsistent
    sysFSI.SetDiscreType(false, false);

    // Set wall boundary condition
    sysFSI.SetWallBC(BceVersion::ADAMI);

    // Setup the solver based on the input value of the prameters
    sysFSI.SetSPHMethod(FluidDynamics::WCSPH);

    // Set the periodic boundary condition
    double initSpace0 = sysFSI.GetInitialSpacing();
    ChVector3d cMin(-bxDim / 2 * 2, -byDim / 2 - 0.5 * iniSpacing, -bzDim * 10);
    ChVector3d cMax( bxDim / 2 * 2,  byDim / 2 + 0.5 * iniSpacing,   bzDim * 20);
    sysFSI.SetBoundaries(cMin, cMax);

    // Set simulation data output length
    sysFSI.SetOutputLength(0);

    // Create an initial box for the terrain patch
    chrono::utils::ChGridSampler<> sampler(initSpace0);
    ChVector3d boxCenter(0, 0, bzDim / 2);
    ChVector3d boxHalfDim(bxDim / 2, byDim / 2, bzDim / 2);
    std::vector<ChVector3d> points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add SPH particles from the sampler points to the FSI system
    std::cout << "Generate SPH particles (" << points.size() << ")" << std::endl;
    auto gz = std::abs(gravity.z());
    int numPart = (int)points.size();
    for (int i = 0; i < numPart; i++) {
        double pre_ini = sysFSI.GetDensity() * gz * (-points[i].z() + bzDim);
        sysFSI.AddSPHParticle(points[i], sysFSI.GetDensity(), 0, sysFSI.GetViscosity(),
                              ChVector3d(0),         // initial velocity
                              ChVector3d(-pre_ini),  // tauxxyyzz
                              ChVector3d(0)          // tauxyxzyz
        );
    }

    // Create MBD and BCE particles for the solid domain
    std::cout << "Generate BCE markers" << std::endl;
    CreateSolidPhase(sysMBS, sysFSI);

    // Complete construction of the FSI system
    sysFSI.Initialize();

    // Get the body from the FSI system for visualization
    std::vector<std::shared_ptr<ChBody>>& FSI_Bodies = sysFSI.GetFsiBodies();

    // Write position and velocity to file
    std::ofstream ofile;
    if (output)
        ofile.open(out_dir + "./body_position.txt");

#if !defined(CHRONO_VSG)
    render = false;
#endif

    std::shared_ptr<ChFsiVisualization> visFSI;
    if (render) {
        #ifdef CHRONO_VSG
                visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
        #endif

        visFSI->SetTitle("Rassor on CRM terrain");
        visFSI->SetSize(1280, 720);
        visFSI->AddCamera(ChVector3d(0, -3 * byDim, bzDim), ChVector3d(0, 0, 0));
        visFSI->SetCameraMoveScale(1.0f);
        visFSI->EnableBoundaryMarkers(true);
        visFSI->EnableRigidBodyMarkers(false);
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetSPHColorCallback(
            chrono_types::make_shared<HeightColorCallback>(ChColor(0.10f, 0.40f, 0.65f), 0, bzDim));
        visFSI->AttachSystem(&sysMBS);
        visFSI->Initialize();
    }

    // Start the simulation
    unsigned int output_steps = (unsigned int)round(1 / (out_fps * dT));
    unsigned int render_steps = (unsigned int)round(1 / (render_fps * dT));
    double time = 0.0;
    int current_step = 0;

    auto body = sysMBS.GetBodies()[1];

    ChTimer timer;
    while (time < total_time) {

        //// RASSOR 2.0, drum spinning counter clock wise
        //driver->SetRazorMotorSpeed((RassorDirID)0, -bucket_driver_speed);
        //driver->SetRazorMotorSpeed((RassorDirID)1,  bucket_driver_speed);

        // RASSOR 1.0, drum spinning  clock wise
        driver->SetDrumMotorSpeed((RassorDirID)0,  bucket_driver_speed);
        driver->SetDrumMotorSpeed((RassorDirID)1, -bucket_driver_speed);


        for (int i = 0; i < 4; i++) {
            driver->SetDriveMotorSpeed((RassorWheelID)i, wheel_driver_speed/3.0);
        }

        ChVector3d front_shoulder_joint_force = rover->GetShoulderMotorReactionForce((RassorDirID)0);
        ChVector3d back_shoulder_joint_force = rover->GetShoulderMotorReactionForce((RassorDirID)1);

        ChVector3d front_shoulder_joint_torque = rover->GetShoulderMotorReactionTorque((RassorDirID)0);
        ChVector3d back_shoulder_joint_torque = rover->GetShoulderMotorReactionTorque((RassorDirID)1);

        ChVector3d rpy = rover->GetChassisRPY();


        // now write the data to the file
        info_file << time << "," << body->GetPos().x() << "," << body->GetPos().y() << "," << body->GetPos().z() << ","
              << rpy.x() << "," << rpy.y() << "," << rpy.z() << ","
			  << front_shoulder_joint_force.x() << "," << front_shoulder_joint_force.y() << "," << front_shoulder_joint_force.z() << ","
			  << front_shoulder_joint_torque.x() << "," << front_shoulder_joint_torque.y() << "," << front_shoulder_joint_torque.z() << ","
			  << back_shoulder_joint_force.x() << "," << back_shoulder_joint_force.y() << "," << back_shoulder_joint_force.z() << ","
			  << back_shoulder_joint_torque.x() << "," << back_shoulder_joint_torque.y() << "," << back_shoulder_joint_torque.z() << std::endl;

        if (time >= 1.0) {
            // raise backdrum
            driver->SetShoulderMotorAngle((RassorDirID)1, 0.25);
        }

        //if (time <= 2.0) {
        //    for (int i = 0; i < 4; i++) {
        //        driver->SetDriveMotorSpeed((RassorWheelID)i, 2.0);
        //    }

        //    driver->SetRazorMotorSpeed((RassorDirID)0, 2.0);
        //    driver->SetRazorMotorSpeed((RassorDirID)1, 2.0);
        //    driver->SetArmMotorSpeed((RassorDirID)0, 0.0);
        //    driver->SetArmMotorSpeed((RassorDirID)1, 0.0);
        //} else if (time > 2.0 && time <= 5.0) {
        //    for (int i = 0; i < 4; i++) {
        //        driver->SetDriveMotorSpeed((RassorWheelID)i, 0.0);
        //    }

        //    driver->SetRazorMotorSpeed((RassorDirID)0, 3.14);
        //    driver->SetRazorMotorSpeed((RassorDirID)1, -3.14);
        //    driver->SetArmMotorSpeed((RassorDirID)0, 0.05);
        //    driver->SetArmMotorSpeed((RassorDirID)1, -0.05);
        //} else if (time > 5.0 && time <= 7.0) {
        //    for (int i = 0; i < 4; i++) {
        //        driver->SetDriveMotorSpeed((RassorWheelID)i, 0.0);
        //    }

        //    driver->SetRazorMotorSpeed((RassorDirID)0, 0.0);
        //    driver->SetRazorMotorSpeed((RassorDirID)1, 0.0);
        //    driver->SetArmMotorSpeed((RassorDirID)0, -0.55);
        //    driver->SetArmMotorSpeed((RassorDirID)1, 0.55);
        //} else if (time > 7.0 && time <= 10.0) {
        //    for (int i = 0; i < 4; i++) {
        //        driver->SetDriveMotorSpeed((RassorWheelID)i, 0.0);
        //    }

        //    driver->SetRazorMotorSpeed((RassorDirID)0, -2.0);
        //    driver->SetRazorMotorSpeed((RassorDirID)1, 2.0);
        //    driver->SetArmMotorSpeed((RassorDirID)0, 0.0);
        //    driver->SetArmMotorSpeed((RassorDirID)1, 0.0);
        //}


        rover->Update();


            if (output && current_step % output_steps == 0) {
                std::cout << current_step << "  time: " << time << "  sim. time: " << timer() << std::endl;
                std::cout << "  pos:            " << body->GetPos() << std::endl;
                std::cout << "  rpy:            " << rpy.x() / CH_PI * 180. << ", " << rpy.y() / CH_PI * 180.
                          << ", " << rpy.z() / CH_PI * 180. << std::endl;   
                std::cout << "  front shoulder: [" << front_shoulder_joint_force << "] N, ["
                          << front_shoulder_joint_torque << "] mN" << std::endl;
                std::cout << "  back shoulder:  [" << back_shoulder_joint_force << "] N, ["
                          << back_shoulder_joint_torque << "] mN" << std::endl;

                sysFSI.PrintParticleToFile(out_dir + "/particles");
                sysFSI.PrintFsiInfoToFile(out_dir + "/fsi", time);
                rover->writeMeshFile(out_dir + "/rover", int(current_step/output_steps), true);
            }

        // Render system
        if (render && current_step % render_steps == 0) {
            if (!visFSI->Render())
                break;
        }

        timer.start();
        sysFSI.DoStepDynamics_FSI();
        timer.stop();

        time += dT;
        current_step++;
    }

    if (output){
		ofile.close();
        info_file.close();
	}

    return 0;
}

//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies and their
// BCE representations are created and added to the systems
//------------------------------------------------------------------
void CreateSolidPhase(ChSystemNSC& sysMBS, ChSystemFsi& sysFSI) {
    // Create a body for the rigid soil container
    auto box = chrono_types::make_shared<ChBodyEasyBox>(10, 10, 0.02, 1000, false, false);
    box->SetPos(ChVector3d(0, 0, 0));
    box->SetFixed(true);
    sysMBS.Add(box);

    // Get the initial SPH particle spacing
    double initSpace0 = sysFSI.GetInitialSpacing();

    // Fluid-Solid Coupling at the walls via BCE particles
    sysFSI.AddBoxContainerBCE(box,                                        //
                              ChFrame<>(ChVector3d(0, 0, bzDim), QUNIT),  //
                              ChVector3d(bxDim, byDim, 2 * bzDim),        //
                              ChVector3i(2, 0, -1));

    driver = chrono_types::make_shared<RassorSpeedDriver>(1.0);
    rover = chrono_types::make_shared<Rassor>(&sysMBS, wheel_type);
    rover->SetDriver(driver);
    rover->SetWheelContactMaterial(CustomWheelMaterial(ChContactMethod::NSC));
    rover->Initialize(ChFrame<>(init_loc, QUNIT));


    std::vector<ChVector3d> BCE_wheel;
    BCE_wheel = LoadSolidPhaseBCE(GetChronoDataFile("robot/rassor/bce/rassor_wheel.csv"));
    std::cout << "BCE wheel len:" << BCE_wheel.size() << std::endl;

    // Add BCE particles and mesh of wheels to the system
    for (int i = 0; i < 4; i++) {
        std::shared_ptr<ChBodyAuxRef> wheel_body;
        if (i == 0) {
            wheel_body = rover->GetWheel(RassorWheelID::RA_LF)->GetBody();
        }
        if (i == 1) {
            wheel_body = rover->GetWheel(RassorWheelID::RA_RF)->GetBody();
        }
        if (i == 2) {
            wheel_body = rover->GetWheel(RassorWheelID::RA_LB)->GetBody();
        }
        if (i == 3) {
            wheel_body = rover->GetWheel(RassorWheelID::RA_RB)->GetBody();
        }

        sysFSI.AddFsiBody(wheel_body);
        if (i == 0 || i == 2) {
            sysFSI.AddPointsBCE(wheel_body, BCE_wheel, ChFrame<>(VNULL, QuatFromAngleZ(CH_PI)), true);
        } else {
            sysFSI.AddPointsBCE(wheel_body, BCE_wheel, ChFrame<>(VNULL, QUNIT), true);
        }
    }

    std::vector<ChVector3d> BCE_razor_back;
    BCE_razor_back = LoadSolidPhaseBCE(GetChronoDataFile("robot/rassor/bce/rassor_drum.csv"));
    std::cout << "BCE Razor len:" << BCE_razor_back.size() << std::endl;

    // now create vector BCE_razor_front, it is the mirrored version of BCE_razor_back, where x value is flipped
    std::vector<ChVector3d> BCE_razor_front;
    for (int i = 0; i < BCE_razor_back.size(); i++) {
        BCE_razor_front.push_back(ChVector3d(-BCE_razor_back[i].x(), BCE_razor_back[i].y(), BCE_razor_back[i].z()));
    }

    // Add BCE particles and mesh of razor to the system
    for (int i = 0; i < 2; i++) {
        std::shared_ptr<ChBodyAuxRef> razor_body;
        if (i == 0) {
            razor_body = rover->GetDrum(RassorDirID::RA_F)->GetBody();
        }
        if (i == 1) {
            razor_body = rover->GetDrum(RassorDirID::RA_B)->GetBody();
        }

        sysFSI.AddFsiBody(razor_body);

        // This is the case for bucket pushing soil away (front drum spin counter clock wise)
        //if (i == 0) {
        //    sysFSI.AddPointsBCE(razor_body, BCE_razor_front, ChFrame<>(VNULL, QUNIT), true);
        //} else {
        //    sysFSI.AddPointsBCE(razor_body, BCE_razor_back, ChFrame<>(VNULL, QUNIT), true);
        //}

        // This is the case for front drum spins clockwise
        if (i == 0) {
            sysFSI.AddPointsBCE(razor_body, BCE_razor_back, ChFrame<>(VNULL, QUNIT), true);
        } else {
            sysFSI.AddPointsBCE(razor_body, BCE_razor_front, ChFrame<>(VNULL, QUNIT), true);
        }



    }
}

std::vector<ChVector3d> LoadSolidPhaseBCE(std::string filename) {
    std::ifstream file(filename);
    std::vector<ChVector3d> points;
    std::string line;

    // Check if the file is open
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return points;
    }

    // Skip the header
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::vector<float> point;
        std::string value;

        while (std::getline(ss, value, ',')) {
            point.push_back(std::stof(value));
        }

        ChVector3d pt_vec;
        pt_vec.x() = point[0];
        pt_vec.y() = point[1];
        pt_vec.z() = point[2];

        points.push_back(pt_vec);
    }

    return points;
}

std::shared_ptr<ChContactMaterial> CustomWheelMaterial(ChContactMethod contact_method) {
    float mu = 0.4f;   // coefficient of friction
    float cr = 0.2f;   // coefficient of restitution
    float Y = 2e7f;    // Young's modulus
    float nu = 0.3f;   // Poisson ratio
    float kn = 2e5f;   // normal stiffness
    float gn = 40.0f;  // normal viscous damping
    float kt = 2e5f;   // tangential stiffness
    float gt = 20.0f;  // tangential viscous damping

    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChContactMaterialNSC>();
            matNSC->SetFriction(mu);
            matNSC->SetRestitution(cr);
            return matNSC;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChContactMaterialSMC>();
            matSMC->SetFriction(mu);
            matSMC->SetRestitution(cr);
            matSMC->SetYoungModulus(Y);
            matSMC->SetPoissonRatio(nu);
            matSMC->SetKn(kn);
            matSMC->SetGn(gn);
            matSMC->SetKt(kt);
            matSMC->SetGt(gt);
            return matSMC;
        }
        default:
            return std::shared_ptr<ChContactMaterial>();
    }
}

bool CreateSubDirectories(std::string out_dir){
        // Create oputput subdirectories
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return false;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
        std::cerr << "Error creating directory " << out_dir + "/particles" << std::endl;
        return false;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/fsi"))) {
        std::cerr << "Error creating directory " << out_dir + "/fsi" << std::endl;
        return false;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/rover"))) {
        std::cerr << "Error creating directory " << out_dir + "/rover" << std::endl;
        return false;
    }

    return true;
}