# Monitoring time
import time
time_start = time.time()

# Loading previous solutions
from equations_of_motion_solution import *
from sympy.physics.vector import vlatex
from sympy import init_printing

# Import NumPy functions to setp the numerical values and integrate the equations of motion
from numpy import deg2rad, rad2deg, array, zeros, linspace

# Import ODE numerical integration routine from SciPy
from scipy.integrate import odeint

# Import PyDy's ODE function generation to generate the symbolic equations into numerical function
from pydy.codegen.ode_function_generators import generate_ode_function

# Import a few functions from Matplotlib.PyPlot
from matplotlib.pyplot import plot, legend, xlabel, ylabel, rcParams

constants = [lower_arm_length,               # Symbolic form
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

################################ Solution that works
specifieds = [lambda1, lambda2, lambda3, lambda4, lambda5]

speeds = [omega1, omega2, omega3, omega4, omega5]

time_span = 10

time_points = 60*10

import numpy as np

t = np.linspace(0,time_span,time_points, dtype=np.float64)

# Creating the torques list
torques = [ground_joint_torque, lower_arm_joint_torque, upper_arm_joint_torque, finger1_joint_torque, finger2_joint_torque]

# Passing the RHS to the ODE function generator
right_hand_side = generate_ode_function(forcing_vector, torques, speeds, constants, mass_matrix=mass_matrix, specifieds=specifieds)

# Initial conditions 
x0 = np.zeros(10)

numerical_constants = array([1,        # lower_arm_length [m]
                             0.44309,  # lower_arm_com_length [m]
                             347.2,    # lower_arm_mass [kg]
                             49.945,   # lower_arm_inertia [kg*m^2]
                             1.62475,  # upper_arm_length [m]
                             0.56774,  # upper_arm_com_length
                             61.10,    # upper_arm_mass [kg]
                             18.80,    # upper_leg_inertia [kg*m^2]
                             0.6428,   # hand_length [m]
                             0.1424,   # hand_com_length [m]
                             7.33,     # hand_mass [kg]
                             0.31035,  # hand_inertia [kg*m^2]
                             0.4218,   # finger1_length [m]
                             0.2077,   # finger1_com_length [m]
                             0.5,      # finger1_mass [kg]
                             0.007322, # finger1_inertia [kg*m^2]
                             0.4218,   # finger2_length [m]
                             0.2077,   # finger2_com_length [m]
                             0.5,      # finger2_mass [kg]
                             0.007322, # finger2_inertia [kg*m^2
                             9.81], dtype=np.float64)   # acceleration due to gravity [m/s^2]

# Create a numpy array of the numerical specified torques
def coordinates(t):
    return -t/1000

numerical_specified =  np.array([1*coordinates(t), 
                                 2*coordinates(t), 
                                 3*coordinates(t), 
                                 4*coordinates(t), 
                                 5*coordinates(t)], dtype=np.float64)

numerical_specified = np.reshape(numerical_specified,
                                (np.shape(numerical_specified)[1], np.shape(numerical_specified)[0]))

time_end = time.time()

print(f'Lapsed time: {time_end-time_start:.2f}s')

#print(right_hand_side(x0, 10.0, numerical_specified[0], numerical_constants))

help(right_hand_side)

# Integrate the ODE 
y = odeint(right_hand_side, x0, t, args=(numerical_specified, numerical_constants))

################################



"""
def l_func(t, tspan, lambda_final, lambda_initial): # Generalized coordinate
    
    t = time vector (numpy array)
    tspan = total time span of the simulation (int)
    lambda_final = final degree (int)
    lambda_initial = initial degree (int)
    
    return np.deg2rad(t*(lambda_final-lambda_initial)/time_span+lambda_initial)

def o_func(tspan, lambda_final, lambda_initial): # Velocities
    
    tspan = total time span of the simulation (int)
    lambda_final = final degree (int)
    lambda_initial = initial degree (int)
    
    return np.deg2rad(lambda_final-lambda_initial)/time_span


specified = [lambda1,                       # Symbolic form
             lambda2,
             lambda3,
             lambda4,
             lambda5]"""


"""numerical_specified = array([[l_func(t, time_span, 45, 80)],
                             [l_func(t, time_span, 45, 80)],
                             [l_func(t, time_span, 45, 80)],
                             [l_func(t, time_span, 45, 80)],
                             [l_func(t, time_span, 45, 80)]],)"""
