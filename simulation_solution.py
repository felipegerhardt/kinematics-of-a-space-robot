# Loading previous solutions
from equations_of_motion_solution import *

# Import System to integrate EOM using Kane's Method
from pydy.system import System

# Import NumPy functions to setp the numerical values and integrate the equations of motion
from numpy import deg2rad, rad2deg, array, zeros, linspace

# Import generate_ode_function to transform symbolic equations to numerical functions
from pydy.codegen.ode_function_generators import generate_ode_function

# Import a few functions from Matplotlib.PyPlot
from matplotlib.pyplot import plot, legend, xlabel, ylabel, rcParams

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
time_span = 3  
time_points = 60*time_span
import numpy as np
t = np.linspace(0,time_span,time_points, dtype=np.float64)

# Passing the RHS to the ODE function generator
right_hand_side = generate_ode_function(forcing_vector, coordinates, speeds, constants, mass_matrix=mass_matrix, specifieds=torques)

# Initial conditions 
x0 = {lambda1: np.deg2rad(-90),                      # Lower arm horizontally oriented
      lambda2: np.deg2rad(0),
      lambda3: np.deg2rad(0),
      lambda4: np.deg2rad(15),
      lambda5: np.deg2rad(-15),
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
numerical_specifieds = [{ground_joint_torque: lambda lambda1, t: -1000*t**2+9000*np.ones(np.shape(t)),
                        lower_arm_joint_torque: lambda lambda2, t: -300*t**2+2700*np.ones(np.shape(t)),
                        upper_arm_joint_torque: lambda lambda3, t: -15*t**2+135*np.ones(np.shape(t)),
                        finger1_joint_torque: lambda lambda4, t: -5*t**2+45*np.ones(np.shape(t)),
                        finger2_joint_torque: lambda lambda5, t: -5*t**2+45*np.ones(np.shape(t))},

                        {ground_joint_torque: lambda lambda1, t: -378*t**2+3404*np.ones(np.shape(t)),
                        lower_arm_joint_torque: lambda lambda2, t: -150*t**2+1350*np.ones(np.shape(t)),
                        upper_arm_joint_torque: lambda lambda3, t: -7*t**2+63*np.ones(np.shape(t)),
                        finger1_joint_torque: lambda lambda4, t: -2*t**2+18*np.ones(np.shape(t)),
                        finger2_joint_torque: lambda lambda5, t: 2*t**2+18*np.ones(np.shape(t))},

                        {ground_joint_torque: lambda lambda1, t: -150*t**2+1350*np.ones(np.shape(t)),
                        lower_arm_joint_torque: lambda lambda2, t: -82.5*t**2+742.5*np.ones(np.shape(t)),
                        upper_arm_joint_torque: lambda lambda3, t: -4*t**2+36*np.ones(np.shape(t)),
                        finger1_joint_torque: lambda lambda4, t: -1*t**2+9*np.ones(np.shape(t)),
                        finger2_joint_torque: lambda lambda5, t: -1*t**2+9*np.ones(np.shape(t))}]

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


       # Numerical specifieds
      numerical_specifieds = [{ground_joint_torque: lambda lambda1, t: -1000*t**2+9000*np.ones(np.shape(t)),
                              lower_arm_joint_torque: lambda lambda2, t: -300*t**2+2700*np.ones(np.shape(t)),
                              upper_arm_joint_torque: lambda lambda3, t: -15*t**2+135*np.ones(np.shape(t)),
                              finger1_joint_torque: lambda lambda4, t: -5*t**2+45*np.ones(np.shape(t)),
                              finger2_joint_torque: lambda lambda5, t: -5*t**2+45*np.ones(np.shape(t))},

                              {ground_joint_torque: lambda lambda1, t: -378*t**2+3404*np.ones(np.shape(t)),
                              lower_arm_joint_torque: lambda lambda2, t: -150*t**2+1350*np.ones(np.shape(t)),
                              upper_arm_joint_torque: lambda lambda3, t: -7*t**2+63*np.ones(np.shape(t)),
                              finger1_joint_torque: lambda lambda4, t: -2*t**2+18*np.ones(np.shape(t)),
                              finger2_joint_torque: lambda lambda5, t: 2*t**2+18*np.ones(np.shape(t))},

                              {ground_joint_torque: lambda lambda1, t: -150*t**2+1350*np.ones(np.shape(t)),
                              lower_arm_joint_torque: lambda lambda2, t: -82.5*t**2+742.5*np.ones(np.shape(t)),
                              upper_arm_joint_torque: lambda lambda3, t: -4*t**2+36*np.ones(np.shape(t)),
                              finger1_joint_torque: lambda lambda4, t: -1*t**2+9*np.ones(np.shape(t)),
                              finger2_joint_torque: lambda lambda5, t: -1*t**2+9*np.ones(np.shape(t))}]
      
      for j in range(len(numerical_specifieds)):
                 
            # Integrating the system with the constants, specified and initial condition values
            sys = System(kane,
                        constants=numerical_constants,
                        specifieds=numerical_specifieds[j],
                        initial_conditions=x0,
                        times=t)
                        
            solutions.append(sys.integrate())