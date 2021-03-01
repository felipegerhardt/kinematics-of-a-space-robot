from inertial_properties_solution import *
print('Inertial properties file loaded')
from sympy.physics.mechanics import RigidBody

# Create the RigidBody object for each body
lower_arm = RigidBody('Lower Arm', lower_arm_mass_center, lower_arm_frame, 
                     lower_arm_mass, lower_arm_central_inertia)
upper_arm = RigidBody('Upper Arm', upper_arm_mass_center, upper_arm_frame, 
                     upper_arm_mass, upper_arm_central_inertia)
hand = RigidBody('Hand', hand_mass_center, hand_frame, 
                     hand_mass, hand_central_inertia)
finger1 = RigidBody('Finger 1', finger1_mass_center, finger1_frame, 
                     finger1_mass, finger1_central_inertia)
finger2 = RigidBody('Finger 2', finger2_mass_center, finger2_frame, 
                     finger2_mass, finger2_central_inertia)

# Create a constant symbol to gravity acceleration
g = symbols('g')

# Express the force acting on the mass center of each body
lower_arm_grav_force_vector = -lower_arm_mass * g * inertial_frame.y
upper_arm_grav_force_vector = -upper_arm_mass * g * inertial_frame.y
hand_grav_force_vector = -hand_mass * g * inertial_frame.y
finger1_grav_force_vector = -finger1_mass * g * inertial_frame.y
finger2_grav_force_vector = -finger2_mass * g * inertial_frame.y

# Create a tuple to hold gravity force information for each body
lower_arm_grav_force = (lower_arm_mass_center, lower_arm_grav_force_vector)
upper_arm_grav_force = (upper_arm_mass_center, upper_arm_grav_force_vector)
hand_grav_force = (hand_mass_center, hand_grav_force_vector)
finger1_grav_force = (finger1_mass_center, finger1_grav_force_vector)
finger2_grav_force = (finger2_mass_center, finger2_grav_force_vector)

# Create the torque dynamic symbols
ground_joint_torque, lower_arm_joint_torque, upper_arm_joint_torque, finger1_joint_torque, finger2_joint_torque = dynamicsymbols('T_g, T_l, T_u, T_f1, T_f2')

# Express the torque vector of each body (remember Newton's third law)
lower_arm_torque_vector = ground_joint_torque * inertial_frame.z - lower_arm_joint_torque * inertial_frame.z
upper_arm_torque_vector = lower_arm_joint_torque * inertial_frame.z - upper_arm_joint_torque * inertial_frame.z
hand_torque_vector = upper_arm_joint_torque * inertial_frame.z - finger1_joint_torque * inertial_frame.z  - finger2_joint_torque * inertial_frame.z 
finger1_torque_vector = finger1_joint_torque * inertial_frame.z
finger2_torque_vector = finger2_joint_torque * inertial_frame.z

# Create a tuple to hold torque information
lower_arm_torque = (lower_arm_frame, lower_arm_torque_vector)
upper_arm_torque = (upper_arm_frame, upper_arm_torque_vector)
hand_torque = (hand_frame, hand_torque_vector)
finger1_torque = (finger1_frame, finger1_torque_vector)
finger2_torque = (finger2_frame, finger2_torque_vector)

from sympy import trigsimp
from sympy.physics.mechanics import KanesMethod

# Create angular velocities dynamicsymbols
omega1, omega2, omega3, omega4, omega5 = dynamicsymbols('omega1, omega2, omega3, omega4, omega5')

# Expliciting the relationship between the generalized coordinates and the angular velocities
kinematical_differential_equations = [omega1 - lambda1.diff(),
                                      omega2 - lambda2.diff(),
                                      omega3 - lambda3.diff(),
                                      omega4 - lambda4.diff(),
                                      omega5 - lambda5.diff()]

# Create a list to hold the generalized coordinates and speeds 
coordinates = [lambda1, lambda2, lambda3, lambda4, lambda5]
speeds = [omega1, omega2, omega3, omega4, omega5]

#### Linear velocities

# Initial velocity of the ground point is zero
ground_joint.set_vel(inertial_frame, 0) 

# The mass center and joint points linear velocity of the bodies will be calculated upwards through the rigid body two point equation
lower_arm_mass_center.v2pt_theory(ground_joint, inertial_frame, lower_arm_frame)
lower_arm_mass_center.vel(inertial_frame) # Express the velocities with respect to the inertial reference frame 
lower_arm_joint.v2pt_theory(ground_joint, inertial_frame, lower_arm_frame)
lower_arm_joint.vel(inertial_frame)       

upper_arm_mass_center.v2pt_theory(lower_arm_joint, inertial_frame, upper_arm_frame)
upper_arm_mass_center.vel(inertial_frame) 
upper_arm_joint.v2pt_theory(lower_arm_joint, inertial_frame, upper_arm_frame)
upper_arm_joint.vel(inertial_frame)       

hand_mass_center.v2pt_theory(upper_arm_joint, inertial_frame, hand_frame)
hand_mass_center.vel(inertial_frame)      
hand_joint.v2pt_theory(upper_arm_joint, inertial_frame, hand_frame)
hand_joint.vel(inertial_frame)            

finger1_mass_center.v2pt_theory(hand_joint, inertial_frame, finger1_frame)
finger1_mass_center.vel(inertial_frame)   

finger2_mass_center.v2pt_theory(hand_joint, inertial_frame, finger2_frame)
finger2_mass_center.vel(inertial_frame)   

# Angular velocities of the reference frames
lower_arm_frame.set_ang_vel(inertial_frame, omega1*inertial_frame.z)
upper_arm_frame.set_ang_vel(lower_arm_frame, omega2*inertial_frame.z)
hand_frame.set_ang_vel(upper_arm_frame, omega3*inertial_frame.z)
finger1_frame.set_ang_vel(hand_frame, omega4*inertial_frame.z)
finger2_frame.set_ang_vel(hand_frame, omega5*inertial_frame.z)

# Initilize the KanesMethod object 
kane = KanesMethod(inertial_frame, coordinates, speeds, kinematical_differential_equations)

# List the loads and rigid bodies
loads = [lower_arm_grav_force,
         upper_arm_grav_force,
         hand_grav_force,
         finger1_grav_force,
         finger2_grav_force, 
         lower_arm_torque,
         upper_arm_torque,
         hand_torque,
         finger1_torque,
         finger2_torque]
         
bodies = [lower_arm, upper_arm, hand, finger1, finger2]

# Input the lists on kanes_equation method to ouput fr and frstar
fr, frstar = kane.kanes_equations(bodies, loads)

# Simplify the mass matrix and forcing vector
mass_matrix = trigsimp(kane.mass_matrix_full)
forcing_vector = trigsimp(kane.forcing_full)