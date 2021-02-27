from reference_frame_solution import *
print('Reference frame file loaded')

from sympy.physics.mechanics import Point, dynamicsymbols
from sympy import symbols, simplify

# Creating the Point objects
ground_joint, lower_arm_joint, upper_arm_joint, hand_joint = Point('G'), Point('L'), Point('U'), Point('H')

# Creating the constant symbols 
lower_arm_length, upper_arm_length, hand_length, finger1_length, finger2_length = symbols('l_L, l_U, l_H, l_F1, l_F2')

# Setting position for the joints
lower_arm_joint.set_pos(ground_joint, lower_arm_length * lower_arm_frame.y)
upper_arm_joint.set_pos(lower_arm_joint, upper_arm_length * upper_arm_frame.y)
hand_joint.set_pos(upper_arm_joint, hand_length * hand_frame.y)
