# Loading previous solutions
from simulation_solution import *

# Import System to integrate EOM using Kane's Method
from pydy.system import System



# Numerical constants
numerical_constants = {lower_arm_length: 1.0,         # lower_arm_length [m]   
                       lower_arm_com_length: 0.44309, # lower_arm_com_length [m]
                       lower_arm_mass: 347.2,         # lower_arm_mass [kg]
                       lower_arm_inertia: 49.945,     # lower_arm_inertia [kg*m^2]
                       upper_arm_length: 1.62475,     # upper_arm_length [m]
                       upper_arm_com_length: 0.56774, # upper_arm_com_length [m]
                       upper_arm_mass: 61.10,         # upper_arm_mass [kg]
                       upper_arm_inertia: 18.80,      # upper_leg_inertia [kg*m^2]
                       hand_length: 0.6428,           # hand_length [m]
                       hand_com_length: 0.1424,       # hand_com_length [m]
                       hand_mass: 7.33,               # hand_mass [kg]
                       hand_inertia: 0.31035,         # hand_inertia [kg*m^2]
                       #finger1_length: 0.4218,       # finger1_length [m]                Don't matter/Not in the mass or forcing matrices
                       finger1_com_length: 0.2077,    # finger1_com_length [m]
                       finger1_mass: 0.5,             # finger1_mass [kg]
                       finger1_inertia: 0.007322,     # finger1_inertia [kg*m^2]
                       #finger2_length: 0.4218,       # finger2_length [m]                Don't matter/Not in the mass or forcing matrices
                       finger2_com_length: 0.2077,    # finger2_com_length [m]
                       finger2_mass: 0.5,             # finger2_mass [kg]
                       finger2_inertia: 0.007322,     # finger2_inertia [kg*m^2]
                       g: 9.81}                       # acceleration due to gravity [m/s^2]

# Scene constants
display_constants = {lower_arm_length: 1.0,         # lower_arm_length [m]   
                       lower_arm_com_length: 0.44309, # lower_arm_com_length [m]
                       lower_arm_mass: 347.2,         # lower_arm_mass [kg]
                       lower_arm_inertia: 49.945,     # lower_arm_inertia [kg*m^2]
                       upper_arm_length: 1.62475,     # upper_arm_length [m]
                       upper_arm_com_length: 0.56774, # upper_arm_com_length [m]
                       upper_arm_mass: 61.10,         # upper_arm_mass [kg]
                       upper_arm_inertia: 18.80,      # upper_leg_inertia [kg*m^2]
                       hand_length: 0.6428,           # hand_length [m]
                       hand_com_length: 0.1424,       # hand_com_length [m]
                       hand_mass: 7.33,               # hand_mass [kg]
                       hand_inertia: 0.31035,         # hand_inertia [kg*m^2]
                       finger1_length: 0.4218,       # finger1_length [m]          
                       finger1_com_length: 0.2077,    # finger1_com_length [m]
                       finger1_mass: 0.5,             # finger1_mass [kg]
                       finger1_inertia: 0.007322,     # finger1_inertia [kg*m^2]
                       finger2_length: 0.4218,       # finger2_length [m]          
                       finger2_com_length: 0.2077,    # finger2_com_length [m]
                       finger2_mass: 0.5,             # finger2_mass [kg]
                       finger2_inertia: 0.007322,     # finger2_inertia [kg*m^2]
                       g: 9.81}  

for i in range(30):

    # Numerical specifieds
    numerical_specifieds = {ground_joint_torque: lambda lambda1, t: 100*i,
                            lower_arm_joint_torque: lambda lambda2, t: 70*i,
                            upper_arm_joint_torque: lambda lambda3, t: 20*i,
                            finger1_joint_torque: lambda lambda4, t: 2*i,
                            finger2_joint_torque: lambda lambda5, t: 5*1}

    # Integrating the system with the constants, specified and initial condition values
    sys = System(kane,
                 constants=numerical_constants,
                 specifieds=numerical_specifieds,
                 initial_conditions=x0,
                 times=t)

    y = sys.integrate()

    import pydy.viz
    from pydy.viz.shapes import Cylinder, Sphere
    from pydy.viz.visualization_frame import VisualizationFrame
    from pydy.viz.scene import Scene

    # Joints associated with Sphere objects with 0.1 radius
    ground_joint_shape = Sphere(color='black', radius=0.1) 
    lower_arm_joint_shape = Sphere(color='black', radius=0.1)
    upper_arm_joint_shape = Sphere(color='black', radius=0.1)
    hand_joint_shape = Sphere(color='black', radius=0.1)

    # Create a visualization frame 
    grund_viz_frame = VisualizationFrame(inertial_frame, ground_joint, ground_joint_shape)
    lower_arm_viz_frame = VisualizationFrame(inertial_frame, lower_arm_joint, lower_arm_joint_shape)
    upper_arm_viz_frame = VisualizationFrame(inertial_frame, upper_arm_joint, upper_arm_joint_shape)
    hand_viz_frame = VisualizationFrame(inertial_frame, hand_joint, hand_joint_shape)

    # Creating Point objects
    lower_arm_center = Point('l_c')
    upper_arm_center = Point('u_c')
    hand_center = Point('t_c')
    finger1_center = Point('t_f1') 
    finger2_center = Point('t_f2')

    # Setting geometrical positions for the bodies
    lower_arm_center.set_pos(ground_joint, lower_arm_length / 2 * lower_arm_frame.y)
    upper_arm_center.set_pos(lower_arm_joint, upper_arm_length / 2 * upper_arm_frame.y)
    hand_center.set_pos(upper_arm_joint, hand_length / 2 * hand_frame.y)
    finger1_center.set_pos(hand_joint, finger1_length * finger1_frame.y)
    finger2_center.set_pos(hand_joint, finger2_length * finger2_frame.y)

    # Create a dictionary to get easy access.
    constants_dict = dict(zip(constants, display_constants))

    # Lower arm shape and visualization frame
    lower_arm_shape = Cylinder(radius=0.08, length=constants_dict[lower_arm_length], color='blue')
    lower_arm_viz_frame = VisualizationFrame('Lower Arm', lower_arm_frame, lower_arm_center, lower_arm_shape)

    # Upper arm shape and visualization frame
    upper_arm_shape = Cylinder(radius=0.08, length=constants_dict[upper_arm_length], color='red')
    upper_arm_viz_frame = VisualizationFrame('Upper Arm', upper_arm_frame, upper_arm_center, upper_arm_shape)

    # Hand shape and visualization frame
    hand_shape = Cylinder(radius=0.08, length=constants_dict[hand_length], color='green')
    hand_viz_frame = VisualizationFrame('Hand', hand_frame, hand_center, hand_shape)

    # Finger 1 shape and visualization frame
    finger1_shape = Cylinder(radius=0.08, length=constants_dict[finger1_length], color='black')
    finger1_viz_frame = VisualizationFrame('Finger 1', finger1_frame, finger1_center, finger1_shape)

    # Finger 2 and visualization frame
    finger2_shape = Cylinder(radius=0.08, length=constants_dict[finger2_length], color='black')
    finger2_viz_frame = VisualizationFrame('Finger 2', finger2_frame, finger2_center, finger2_shape)

    scene = Scene(inertial_frame, ground_joint)

    # Append the frames to be visualized in a list.
    scene.visualization_frames = [grund_viz_frame,
                                  lower_arm_viz_frame, 
                                  upper_arm_viz_frame, 
                                  hand_viz_frame, 
                                  lower_arm_viz_frame,
                                  upper_arm_viz_frame,
                                  hand_viz_frame,
                                  finger1_viz_frame,
                                  finger2_viz_frame]

    scene.states_symbols = coordinates + speeds
    scene.constants = display_constants
    scene.states_trajectories = y

    scene.display()
