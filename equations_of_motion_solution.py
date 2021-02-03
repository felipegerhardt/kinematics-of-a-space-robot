from inertial_properties_solution import *
from sympy.physics.mechanics import RigidBody

# Lower arm rigid body
lower_arm = RigidBody('Lower Arm', lower_arm_mass_center, lower_arm_frame, 
                     lower_arm_mass, lower_arm_central_inertia)

# Upper arm rigid body
upper_arm = RigidBody('Upper Arm', upper_arm_mass_center, upper_arm_frame, 
                     upper_arm_mass, upper_arm_central_inertia)

# Hand rigid body
hand = RigidBody('Hand', hand_mass_center, hand_frame, 
                     hand_mass, hand_central_inertia)

# Finger1 rigid body
finger1 = RigidBody('Finger 1', finger1_mass_center, finger1_frame, 
                     finger1_mass, finger1_central_inertia)

# Finge2 rigid body
finger2 = RigidBody('Finger 2', finger2_mass_center, finger2_frame, 
                     finger2_mass, finger2_central_inertia)

g = symbols('g')

# Force acting on the lower arm mass center
lower_arm_grav_force_vector = -lower_arm_mass * g * inertial_frame.y

# Force acting on the upper arm mass center
upper_arm_grav_force_vector = -upper_arm_mass * g * inertial_frame.y

# Force acting on the hand mass center
hand_grav_force_vector = -hand_mass * g * inertial_frame.y

# Force acting on the finger 1 mass center
finger1_grav_force_vector = -finger1_mass * g * inertial_frame.y

# Force acting on the finger2 mass center
finger2_grav_force_vector = -finger2_mass * g * inertial_frame.y

# Lower arm gravity force tuple
lower_arm_grav_force = (lower_arm_mass_center, lower_arm_grav_force_vector)

# Upper arm gravity force tuple
upper_arm_grav_force = (upper_arm_mass_center, upper_arm_grav_force_vector)

# Hand gravity force tuple
hand_grav_force = (hand_mass_center, hand_grav_force_vector)

# Finger 1 gravity force tuple
finger1_grav_force = (finger1_mass_center, finger1_grav_force_vector)

# Finger 2 gravity force tuple
finger2_grav_force = (finger2_mass_center, finger2_grav_force_vector)

# Creating torque symbols
ground_joint_torque, lower_arm_joint_torque, upper_arm_joint_torque, finger1_joint_torque, finger2_joint_torque = dynamicsymbols('T_g, T_l, T_u, T_f1, T_f2')

# Lower arm torques
lower_arm_torque_vector = ground_joint_torque * inertial_frame.z - lower_arm_joint_torque * inertial_frame.z

# Upper arm torques
upper_arm_torque_vector = lower_arm_joint_torque * inertial_frame.z - upper_arm_joint_torque * inertial_frame.z

# Hand torques
hand_torque_vector = upper_arm_joint_torque * inertial_frame.z + finger1_joint_torque * inertial_frame.z  - finger2_joint_torque * inertial_frame.z 

# Finger 1 torque
finger1_torque_vector = finger1_joint_torque * inertial_frame.z

# Figer 2 torque
finger2_torque_vector = finger2_joint_torque * inertial_frame.z

# Lower arm torque tuple
lower_arm_torque = (lower_arm_frame, lower_arm_torque_vector)

# Upper arm torque tuple 
upper_arm_torque = (upper_arm_frame, lower_arm_torque_vector)

# Hand torque tuple
hand_torque = (hand_frame, hand_torque_vector)

# Finger 1 torque tuple
finger1_torque = (finger1_frame, finger1_torque_vector)

# Finger 2 torque tuple
finger2_torque = (finger2_frame, finger2_torque_vector)

from sympy import trigsimp
from sympy.physics.mechanics import KanesMethod

# Creating angular velocities dynamicsymbols

omega1, omega2, omega3, omega4, omega5 = dynamicsymbols('omega1, omega2, omega3, omega4, omega5')

# Expliciting the relationship between the generalized coordinates and the angular velocities

kinematical_differential_equations = [omega1 - lambda1.diff(),
                                      omega2 - lambda2.diff(),
                                      omega3 - lambda3.diff(),
                                      omega4 - lambda4.diff(),
                                      omega5 - lambda5.diff()]

coordinates = [lambda1, lambda2, lambda3, lambda4, lambda5]

speeds = [omega1, omega2, omega3, omega4, omega5]

# Linear velocities

# Initial velocity of the ground point is zero
ground_joint.set_vel(inertial_frame, 0) 

# The mass center and joint points linear velocity of the bodies will be calculated upwards through the rigid body two point equation

# Lower arm
lower_arm_mass_center.v2pt_theory(ground_joint, inertial_frame, lower_arm_frame)
lower_arm_mass_center.vel(inertial_frame)

lower_arm_joint.v2pt_theory(ground_joint, inertial_frame, lower_arm_frame)
lower_arm_joint.vel(inertial_frame)

# Upper arm
upper_arm_mass_center.v2pt_theory(lower_arm_joint, inertial_frame, upper_arm_frame)
upper_arm_mass_center.vel(inertial_frame)

upper_arm_joint.v2pt_theory(lower_arm_joint, inertial_frame, upper_arm_frame)
upper_arm_joint.vel(inertial_frame)

# Hand
hand_mass_center.v2pt_theory(upper_arm_joint, inertial_frame, hand_frame)
hand_mass_center.vel(inertial_frame)

hand_joint.v2pt_theory(upper_arm_joint, inertial_frame, hand_frame)
hand_joint.vel(inertial_frame)

# Finger 1
finger1_mass_center.v2pt_theory(hand_joint, inertial_frame, finger1_frame)
finger1_mass_center.vel(inertial_frame)

# Finger 2
finger2_mass_center.v2pt_theory(hand_joint, inertial_frame, finger2_frame)

# Angular velocities
lower_arm_frame.set_ang_vel(inertial_frame, omega1*inertial_frame.z)

upper_arm_frame.set_ang_vel(lower_arm_frame, omega2*inertial_frame.z)

hand_frame.set_ang_vel(upper_arm_frame, omega3*inertial_frame.z)

finger1_frame.set_ang_vel(hand_frame, omega4*inertial_frame.z)

finger2_frame.set_ang_vel(hand_frame, omega5*inertial_frame.z)

# Inputing the KanesMethod, listing the loads and rigid bodies
kane = KanesMethod(inertial_frame, coordinates, speeds, kinematical_differential_equations)

loads = [lower_arm_grav_force,
         upper_arm_grav_force,
         hand_grav_force,
         finger1_grav_force,
         finger2_grav_force, 
         lower_arm_torque,
         upper_arm_torque,
         finger1_torque,
         finger2_torque]
         
bodies = [lower_arm, upper_arm, hand, finger1, finger2]

fr, frstar = kane.kanes_equations(bodies, loads)

mass_matrix = trigsimp(kane.mass_matrix_full)

forcing_vector = trigsimp(kane.forcing_full)