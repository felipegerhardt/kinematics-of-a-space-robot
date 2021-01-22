from sympy.physics.mechanics import ReferenceFrame
from sympy.physics.vector import init_vprinting, dynamicsymbols

inertial_frame, lower_arm_frame, upper_arm_frame = ReferenceFrame('I'), ReferenceFrame('L'), ReferenceFrame('U')
hand_frame, finger1_frame, finger2_frame = ReferenceFrame('H'), ReferenceFrame('F1'), ReferenceFrame('F2')

lambda1, lambda2, lambda3, lambda4, lambda5 = dynamicsymbols('lambda1, lambda2, lambda3, lambda4, lambda5')

lower_arm_frame.orient(inertial_frame, 'Axis', (lambda1, inertial_frame.z))
upper_arm_frame.orient(lower_arm_frame, 'Axis', (lambda2, lower_arm_frame.z))
hand_frame.orient(upper_arm_frame, 'Axis', (lambda3, upper_arm_frame.z))
finger1_frame.orient(hand_frame, 'Axis', (lambda4, hand_frame.z))
finger2_frame.orient(hand_frame, 'Axis', (lambda5, hand_frame.z))
