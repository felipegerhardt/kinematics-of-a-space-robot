from __future__ import print_function, division
from simulation_solution import *
from pydy.viz.shapes import Cylinder, Sphere
import pydy.viz
from pydy.viz.visualization_frame import VisualizationFrame
from pydy.viz.scene import Scene

ground_joint_shape = Sphere(color='black', radius=0.1)
lower_arm_joint_shape = Sphere(color='black', radius=0.1)
upper_arm_joint_shape = Sphere(color='black', radius=0.1)
hand_joint_shape = Sphere(color='black', radius=0.1)

grund_viz_frame = VisualizationFrame(inertial_frame, ground_joint, ground_joint_shape)
lower_arm_viz_frame = VisualizationFrame(inertial_frame, lower_arm_joint, lower_arm_joint_shape)
upper_arm_viz_frame = VisualizationFrame(inertial_frame, upper_arm_joint, upper_arm_joint_shape)
hand_viz_frame = VisualizationFrame(inertial_frame, hand_joint, hand_joint_shape)


lower_arm_center = Point('l_c')
upper_arm_center = Point('u_c')
hand_center = Point('t_c')
finger1_center = Point('t_f1')
finger2_center = Point('t_f2')

lower_arm_center.set_pos(ground_joint, lower_arm_length / 2 * lower_arm_frame.y)
upper_arm_center.set_pos(lower_arm_joint, upper_arm_length / 2 * upper_arm_frame.y)
hand_center.set_pos(upper_arm_joint, hand_length / 2 * hand_frame.y)
finger1_center.set_pos(hand_joint, finger1_length * finger1_frame.y)
finger2_center.set_pos(hand_joint, finger2_length * finger2_frame.y)

constants_dict = dict(zip(constants, numerical_constants))


# Lower arm shape and visualization frame
lower_arm_shape = Cylinder(radius=0.08, length=constants_dict[lower_arm_length], color='blue')
lower_arm_viz_frame = VisualizationFrame('Lower Arm', lower_arm_frame, lower_arm_center, lower_arm_shape)

# Upper arm shape and visualization frame
upper_arm_shape = Cylinder(radius=0.08, length=constants_dict[upper_arm_length], color='blue')
upper_arm_viz_frame = VisualizationFrame('Upper Arm', upper_arm_frame, upper_arm_center, upper_arm_shape)

# Hand shape and visualization frame
hand_shape = Cylinder(radius=0.08, length=constants_dict[hand_length], color='blue')
hand_viz_frame = VisualizationFrame('Hand', hand_frame, hand_center, hand_shape)

# Finger 1 shape and visualization frame
finger1_shape = Cylinder(radius=0.08, length=constants_dict[finger1_length], color='blue')
finger1_viz_frame = VisualizationFrame('Finger 1', finger1_frame, finger1_center, finger1_shape)

# Finger 2 and visualization frame
finger2_shape = Cylinder(radius=0.08, length=constants_dict[finger2_length], color='blue')
finger2_viz_frame = VisualizationFrame('Finger 2', finger2_frame, finger2_center, finger2_shape)

scene = Scene(inertial_frame, ground_joint)


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
scene.constants = constants_dict
scene.states_trajectories = y

scene.display()