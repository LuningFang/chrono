# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2014 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================

print ("Example: Load a STEP file, generated by a CAD.");
print ("Please wait! this may take a while to load the file..."); 

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.cascade as cascade
from OCC.Core import TopoDS

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

#  Create the simulation system and add items
sys = chrono.ChSystemNSC()

# Load a STEP file, containing a mechanism. The demo STEP file has been
# created using a 3D CAD (in this case, SolidEdge v.18).

# Create the ChCascadeDoc, a container that loads the STEP model
# and manages its subassembles
mydoc = cascade.ChCascadeDoc()

# Load the STEP model
load_ok = mydoc.Load_STEP(chrono.GetChronoDataFile('cascade/IRB7600_23_500_m2000_rev1_01_decorated.stp'))  # or specify abs.path: ("C:\\data\\cascade\\assembly.stp");

if not load_ok:
    raise ValueError("Warning. Desired STEP file could not be opened/parsed \n")
      
CH_C_PI = 3.1456

# In most CADs the Y axis is horizontal, but we want it vertical.
# So define a root transformation for rotating all the imported objects.
rotation1 = chrono.ChQuaternionD()
rotation1.Q_from_AngAxis(-CH_C_PI / 2, chrono.ChVectorD(1, 0, 0))  # 1: rotate 90° on X axis
rotation2 = chrono.ChQuaternionD()
rotation2.Q_from_AngAxis(CH_C_PI, chrono.ChVectorD(0, 1, 0))  # 2: rotate 180° on vertical Y axis
tot_rotation = chrono.ChQuaternionD()
tot_rotation = rotation2 % rotation1     # rotate on 1 then on 2, using quaternion product
root_frame = chrono.ChFrameMovingD(chrono.ChVectorD(0, 0, 0), tot_rotation)

# Retrieve some sub shapes from the loaded model, using
# the GetNamedShape() function, that can use path/subpath/subsubpath/part
# syntax and * or ? wildcards, etc.
def make_body_from_name(partname, root_transformation):
    shape1 = TopoDS.TopoDS_Shape()
    if (mydoc.GetNamedShape(shape1, partname)):
        # Make a ChBody representing the TopoDS_Shape part from the CAD:
        mbody1 = cascade.ChCascadeBodyEasy(shape1, # shape
                                           1000,   # density (center of mass & inertia automatically computed)
                                           True,    # mesh for visualization?
                                           False)   # mesh for collision?
        sys.Add(mbody1)
        # Move the body as for global displacement/rotation (also mbody1 %= root_frame; )
        mbody1.ConcatenatePreTransformation(root_transformation)
        return mbody1
    else:
        raise ValueError("Warning. Body part name cannot be found in STEP file.\n")
      
      
rigidBody_base     = make_body_from_name("Assem10/Assem8", root_frame)
rigidBody_turret   = make_body_from_name("Assem10/Assem4", root_frame)
rigidBody_bicep    = make_body_from_name("Assem10/Assem1", root_frame)
rigidBody_elbow    = make_body_from_name("Assem10/Assem5", root_frame)
rigidBody_forearm  = make_body_from_name("Assem10/Assem7", root_frame)
rigidBody_wrist    = make_body_from_name("Assem10/Assem6", root_frame)
rigidBody_hand     = make_body_from_name("Assem10/Assem9", root_frame)
rigidBody_cylinder = make_body_from_name("Assem10/Assem3", root_frame)
rigidBody_rod      = make_body_from_name("Assem10/Assem2", root_frame)

rigidBody_base.SetBodyFixed(True)
#rigidBody_hand.SetBodyFixed(True)

# Create joints between two parts.
# To understand where is the axis of the joint, we can exploit the fact
# that in the STEP file that we prepared for this demo, we inserted some
# objects called 'marker' and we placed them aligned to the shafts, so now
# we can fetch them and get their position/rotation.

def make_frame_from_name(partname, root_transformation):
    shape_marker = TopoDS.TopoDS_Shape()
    if (mydoc.GetNamedShape(shape_marker, partname)):
        frame_marker = chrono.ChFrameD()
        mydoc.FromCascadeToChrono(shape_marker.Location(), frame_marker)
        frame_marker.ConcatenatePreTransformation(root_transformation)
        return frame_marker
    else:
        raise ValueError("Warning. Marker part name cannot be found in STEP file.\n")

frame_marker_base_turret    = make_frame_from_name("Assem10/Assem8/marker#1", root_frame)
frame_marker_turret_bicep   = make_frame_from_name("Assem10/Assem4/marker#2", root_frame)
frame_marker_bicep_elbow    = make_frame_from_name("Assem10/Assem1/marker#2", root_frame)
frame_marker_elbow_forearm  = make_frame_from_name("Assem10/Assem5/marker#2", root_frame)
frame_marker_forearm_wrist  = make_frame_from_name("Assem10/Assem7/marker#2", root_frame)
frame_marker_wrist_hand     = make_frame_from_name("Assem10/Assem6/marker#2", root_frame)
frame_marker_turret_cylinder= make_frame_from_name("Assem10/Assem4/marker#3", root_frame)
frame_marker_cylinder_rod   = make_frame_from_name("Assem10/Assem3/marker#2", root_frame)
frame_marker_rod_bicep      = make_frame_from_name("Assem10/Assem2/marker#2", root_frame)

                                                   
# Create joints between the parts. 
# This can be done by creating link objects between couples of the bodies
# created in the section above, where the joint position is one of the 
# frame_marker_xxxx_zzzz frames created above.

my_link1 = chrono.ChLinkLockRevolute()
my_link1.Initialize(rigidBody_base, rigidBody_turret, frame_marker_base_turret.GetCoord())
sys.Add(my_link1)
    
my_link2 = chrono.ChLinkLockRevolute()                                               
my_link2.Initialize(rigidBody_turret, rigidBody_bicep, frame_marker_turret_bicep.GetCoord())
sys.Add(my_link2)  
        
my_link3 = chrono.ChLinkLockRevolute()
my_link3.Initialize(rigidBody_bicep, rigidBody_elbow, frame_marker_bicep_elbow.GetCoord())
sys.Add(my_link3)

my_link4 = chrono.ChLinkLockRevolute()
my_link4.Initialize(rigidBody_elbow, rigidBody_forearm, frame_marker_elbow_forearm.GetCoord())
sys.Add(my_link4)

my_link5 = chrono.ChLinkLockRevolute()
my_link5.Initialize(rigidBody_forearm, rigidBody_wrist, frame_marker_forearm_wrist.GetCoord())
sys.Add(my_link5)

my_link6 = chrono.ChLinkLockRevolute()
my_link6.Initialize(rigidBody_wrist, rigidBody_hand, frame_marker_wrist_hand.GetCoord())
sys.Add(my_link6)

my_link7 = chrono.ChLinkLockRevolute()
my_link7.Initialize(rigidBody_turret, rigidBody_cylinder, frame_marker_turret_cylinder.GetCoord())
sys.Add(my_link7)

my_link8 = chrono.ChLinkLockRevolute()
my_link8.Initialize(rigidBody_cylinder, rigidBody_rod, frame_marker_cylinder_rod.GetCoord())
sys.Add(my_link8)
        
my_link9 = chrono.ChLinkLockRevolute()
my_link9.Initialize(rigidBody_rod, rigidBody_bicep, frame_marker_rod_bicep.GetCoord())
sys.Add(my_link9)

                         
# Create a large cube as a floor.

floor = chrono.ChBodyEasyBox(5, 1, 5, 1000, True, True)
floor.SetPos(chrono.ChVectorD(0,-0.5,0))
floor.SetBodyFixed(True)
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/blue.png'))
sys.Add(floor)


# We want to move the hand of the robot using a trajectory.
# Since in this demo all joints are 'off' (i.e just revolute joints),
# it follows that if we move the hand all the robot will automatically
# move as in inverse kinematics.

# Create a ChLinePath geometry, for the hand path, and insert arc/lines sub-paths:
path = chrono.ChLinePath()
a1 = chrono.ChLineArc(
            chrono.ChCoordsysD(rigidBody_hand.GetPos(), # arc center position
                               chrono.Q_ROTATE_X_TO_Z),   # arc plane alignment (default: xy plane) 
            0.3, # radius 
            -chrono.CH_C_PI_2, # start arc ngle (counterclockwise, from local x)
            -chrono.CH_C_PI_2+chrono.CH_C_2PI, # end arc angle 
            True)
path.AddSubLine(a1)
path.SetPathDuration(2)
path.Set_closed(True)

# Create a ChVisualShapeLine, a visualization asset for lines.
pathasset = chrono.ChVisualShapeLine()
pathasset.SetLineGeometry(path)
floor.AddVisualShape(pathasset)

# This is the constraint that uses the trajectory
trajectory = chrono.ChLinkTrajectory()
# Define which parts are connected (the trajectory is considered in the 2nd body).
trajectory.Initialize(rigidBody_hand, # body1 that follows the trajectory
          floor,                 # body2 that 'owns' the trajectory
          chrono.VNULL,           # point on body1 that will follow the trajectory, in body1 coords
          path                   # the trajectory (reuse the one already added to body2 as asset)
          )
sys.Add(trajectory)

# Optionally, set a function that gets the curvilinear
# abscyssa s of the line, as a function of time s(t). 
# By default it was simply  s=t.
spacefx = chrono.ChFunction_Ramp(0, 0.5)
trajectory.Set_space_fx(spacefx)

# Just to constraint the hand rotation:
parallelism = chrono.ChLinkLockParallel()
parallelism.Initialize(rigidBody_hand, floor, frame_marker_wrist_hand.GetCoord())
sys.Add(parallelism);

#  Create an Irrlicht application to visualize the system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Import STEP')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(2,2,2),chrono.ChVectorD(0,0.8,0))
vis.AddTypicalLights()
#vis.AddLightWithShadow(chrono.ChVectorD(3,6,2),  # point
#                       chrono.ChVectorD(0,0,0),  # aimpoint
#                       12,                       # radius (power)
#                       1,11,                     # near, far
#                       55)                       # angle of FOV

#vis.EnableShadows()

# Change the solver form the default SOR to a more precise solver

#solver = mkl.ChSolverMKLcsm()
#sys.SetSolver(solver)

sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN);
#sys.SetSolverType(chrono.ChSolver.Type_MINRES);
sys.SetSolverMaxIterations(300)

# Run the simulation
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)





