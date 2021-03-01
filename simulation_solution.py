# Loading previous solutions
from equations_of_motion_solution import *
print('Equations of motion file loaded')

# Import System to integrate EOM using Kane's Method
from pydy.system import System

# Import NumPy functions to setp the numerical values and integrate the equations of motion
from numpy import deg2rad, rad2deg, array, zeros, linspace, shape, ones

# Import generate_ode_function to transform symbolic equations to numerical functions
from pydy.codegen.ode_function_generators import generate_ode_function

# Creating a list with the symbolic constants
constants = [lower_arm_length,                               # Symbolic form
             lower_arm_com_length,
             lower_arm_mass,
             lower_arm_inertia,
             upper_arm_length,
             upper_arm_com_length,
             upper_arm_mass,
             upper_arm_inertia,
             hand_length,
             hand_com_length,
             hand_mass,            
             hand_inertia,
             finger1_length,
             finger1_com_length,
             finger1_mass,
             finger1_inertia,
             finger2_length,
             finger2_com_length,
             finger2_mass,
             finger2_inertia,           
             g]

# Create lists to hold the torques, coordinates and speeds
torques = [ground_joint_torque, lower_arm_joint_torque, upper_arm_joint_torque, finger1_joint_torque, finger2_joint_torque]
coordinates = [lambda1, lambda2, lambda3, lambda4, lambda5]
speeds = [omega1, omega2, omega3, omega4, omega5]

# Creating time domain as an numpy array
time_span = 0.75
time_points = 60*time_span
t = linspace(0,time_span,time_points)

# Passing the RHS to the ODE function generator
right_hand_side = generate_ode_function(forcing_vector, coordinates, speeds, constants, mass_matrix=mass_matrix, specifieds=torques)

# Initial conditions 
x0 = {lambda1: deg2rad(-90),                      # Lower arm horizontally oriented
      lambda2: deg2rad(0),
      lambda3: deg2rad(0),
      lambda4: deg2rad(-15),
      lambda5: deg2rad(15),
      omega1: 0.0,
      omega2: 0.0,
      omega3: 0.0,
      omega4: 0.0,
      omega5: 0.0}

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

# Declare the gravity values for earth, mars and the moon.
g_values = [9.81, 3.711, 1.62] 

# Create the torques list
numerical_specifieds = [{ground_joint_torque: lambda lambda1, t: -7644*t**2+4300*ones(shape(t)), # Generating good results
                        lower_arm_joint_torque: lambda lambda2, t: -1422*t**2+800*ones(shape(t)),
                        upper_arm_joint_torque: lambda lambda3, t: -53*t**2+30*ones(shape(t)),
                        finger1_joint_torque: lambda lambda4, t: -3.55*t**2+1.2*ones(shape(t)),
                        finger2_joint_torque: lambda lambda5, t: -2.66*t**2+1.5*ones(shape(t))},

                        {ground_joint_torque: lambda lambda1, t: -4355.56*t**2+2450*ones(shape(t)),
                        lower_arm_joint_torque: lambda lambda2, t: -755*t**2+425*ones(shape(t)),
                        upper_arm_joint_torque: lambda lambda3, t: -28.44*t**2+16*ones(shape(t)),
                        finger1_joint_torque: lambda lambda4, t: -0.807*t**2+0.554*ones(shape(t)),
                        finger2_joint_torque: lambda lambda5, t: -1*t**2+0.6674*ones(shape(t))},

                        {ground_joint_torque: lambda lambda1, t: -3022.22*t**2+1700*ones(shape(t)),
                        lower_arm_joint_torque: lambda lambda2, t: -533.33*t**2+300*ones(shape(t)),
                        upper_arm_joint_torque: lambda lambda3, t: -17.78*t**2+10*ones(shape(t)),
                        finger1_joint_torque: lambda lambda4, t: -0.5511*t**2+0.31*ones(shape(t)),
                        finger2_joint_torque: lambda lambda5, t: -0.8533*t**2+0.48*ones(shape(t))}]

# Create the solutions list                  
solutions = []

for i in range(len(g_values)):

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
                             g: g_values[i]}                # acceleration due to gravity [m/s^2]
      
      for j in range(len(numerical_specifieds)):
                 
            # Integrating the system with the constants, specified and initial condition values
            sys = System(kane,
                        constants=numerical_constants,
                        specifieds=numerical_specifieds[j],
                        initial_conditions=x0,
                        times=t)
                        
            solutions.append(sys.integrate())
            print(f'Simulation (g: {g_values[i]} m/s², torques comb.: {j+1}) completed')