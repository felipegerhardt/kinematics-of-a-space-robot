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

constants = [lower_arm_length,
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
coordinates = [lambda1, lambda2, lambda3, lambda4, lambda5]

speeds = [omega1, omega2, omega3, omega4, omega5]

specifieds = [ground_joint_torque, lower_arm_joint_torque, upper_arm_joint_torque, finger1_joint_torque, finger2_joint_torque]

# Passing the RHS to the ODE function generator
right_hand_side = generate_ode_function(forcing_vector, coordinates, speeds, constants, mass_matrix=mass_matrix, specifieds=specifieds)

# Setting the speeds and coordinates initial conditions 
x0 = zeros(10)

# Speeds initial conditions are zero, while the lower arm, upper arm and hand generalized coordinates are on an 20° angle. The fingers are on a 20° angle with respect to the hand axis.
x0[5:8] = deg2rad(0)
x0[8] = deg2rad(0) # Bottom finger
x0[9] = deg2rad(0) # Top finger

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
                             9.81],    # acceleration due to gravity [m/s^2]
                            )

import numpy as np
t = np.linspace(0,10,60*10)

numerical_specified = [0,10,0,0,0,0]

right_hand_side(x0, 0.0, numerical_specified, numerical_constants)

y = odeint(right_hand_side, x0, t, args=(numerical_specified, numerical_constants))