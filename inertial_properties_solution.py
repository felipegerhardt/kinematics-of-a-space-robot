# Loading previous solutions
from points_location_solution import *
from sympy.physics.mechanics import Point, dynamicsymbols, inertia, RigidBody
from sympy import symbols, simplify

# Creating points objects
lower_arm_mass_center, upper_arm_mass_center, hand_mass_center, finger1_mass_center, finger2_mass_center = Point('L_o'), Point('U_o'), Point('H_o'), Point('F1_o'), Point('F2_o') 

# Creating lengths symbols
lower_arm_com_length, upper_arm_com_length, hand_com_length, finger1_com_length, finger2_com_length = symbols('d_L, d_U, d_H, d_F1, d_F2') 

# Positioning de mass centers
lower_arm_mass_center.set_pos(ground_joint, lower_arm_com_length * lower_arm_frame.y)
upper_arm_mass_center.set_pos(lower_arm_joint, upper_arm_com_length * upper_arm_frame.y)
hand_mass_center.set_pos(upper_arm_joint, hand_com_length * hand_frame.y)
finger1_mass_center.set_pos(hand_joint, finger1_com_length * finger1_frame.y)
finger2_mass_center.set_pos(hand_joint, finger2_com_length * finger2_frame.y)

# Creating mass and inertia symbolic constants
lower_arm_mass, upper_arm_mass, hand_mass, finger1_mass, finger2_mass = symbols('m_L, m_U, m_H, m_F1, m_F2')
lower_arm_inertia, upper_arm_inertia, hand_inertia, finger1_inertia, finger2_inertia = symbols('I_Lz, I_Uz, I_Hz, I_F1z, I_F2z')

# Lower arm inertia dyadic
lower_arm_inertia_dyadic = inertia(lower_arm_frame, 0, 0, lower_arm_inertia)

# Upper arm inertia dyadic
upper_arm_inertia_dyadic = inertia(upper_arm_frame, 0, 0, upper_arm_inertia)

# Hand inertia dyadic
hand_inertia_dyadic = inertia(hand_frame, 0, 0, hand_inertia)

# Finger1 inertia dyadic
finger1_inertia_dyadic = inertia(finger1_frame, 0, 0, finger1_inertia)

# Finger2 inertia dyadic
finger2_inertia_dyadic = inertia(finger2_frame, 0, 0, finger2_inertia)

# Lower arm inertia tuple
lower_arm_central_inertia = (lower_arm_inertia_dyadic, lower_arm_mass_center)

# Upper arm inertia tuple
upper_arm_central_inertia = (upper_arm_inertia_dyadic, upper_arm_mass_center)

# Hand inertia tuple
hand_central_inertia = (hand_inertia_dyadic, hand_mass_center)

# Finger1 inertia tuple
finger1_central_inertia = (finger1_inertia_dyadic, finger1_mass_center)

# Finger2 inertia tuple
finger2_central_inertia = (finger2_inertia_dyadic, finger2_mass_center)