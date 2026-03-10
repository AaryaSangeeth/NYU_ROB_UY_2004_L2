import math
import numpy as np
import scipy

def forward_kinematics(theta1, theta2, theta3):
    def rotation_x(angle):
        return np.array([
            [1, 0, 0, 0],
            [0, np.cos(angle), -np.sin(angle), 0],
            [0, np.sin(angle), np.cos(angle), 0],
            [0, 0, 0, 1]
        ])

    def rotation_y(angle):
        return np.array([
            [np.cos(angle), 0, np.sin(angle), 0],
            [0, 1, 0, 0],
            [-np.sin(angle), 0, np.cos(angle), 0],
            [0, 0, 0, 1]
        ])

    def rotation_z(angle):
        return np.array([
            [np.cos(angle), -np.sin(angle), 0, 0],
            [np.sin(angle), np.cos(angle), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

    def translation(x, y, z):
        return np.array([
            [1, 0, 0, x],
            [0, 1, 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1]
        ])
    
    T_0_1 = translation(0.07500, -0.08350, 0) @ rotation_x(1.57080) @ rotation_z(theta1)
    T_1_2 = rotation_y(-1.57080) @ rotation_z(theta2)
    T_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(theta3)
    T_3_ee = translation(0.06231, -0.06216, 0.01800)
    T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee
    return T_0_ee[:3, 3]

    



##PART 1 : IK SOLVER 
##helper function for computing error between current ee psotion and target ee positio
def get_error(joint_angles, target_pos):
    # compute current ee position 
    ee_pos = forward_kinematics(joint_angles[0], joint_angles[1], joint_angles[2])
    
    # compute the error as the distance between the end-effector position and the target position
    error = np.linalg.norm(ee_pos - target_pos)
    
    return error

def inverse_kinematics_with_optimizer(ee_position):
    initial_guess = np.array([0.0, 0.0, 0.0])  # Initial guess for joint angles
   
    result = scipy.optimize.minimize(
            fun=get_error,
            x0=initial_guess,
            args=(ee_position,),
            method='Nelder-Mead'
        )
        
    return result.x


##PART 2 : IK SOLVER WITH GRADIENT DESCENT
#helper functions 
def get_cost(joint_angles, ee_position):
    current_ee_pos = forward_kinematics(joint_angles[0], joint_angles[1], joint_angles[2])
    C = np.sum((current_ee_pos - ee_position) ** 2)  # sum of squared error
    mean_error = np.mean(np.abs(current_ee_pos - ee_position))  # mean absolute error
    return C, mean_error
    
def get_gradient(joint_angles, ee_position):
    epsilon = 1e-4
    gradient = np.zeros_like(joint_angles)
    
    #computing the gradient for each joint angle 
    for i in range(len(joint_angles)):
        theta_plus = np.copy(joint_angles)
        theta_minus = np.copy(joint_angles)
        theta_plus[i] += epsilon
        C_plus, _ = get_cost(theta_plus, ee_position)
        
        
        theta_minus[i] -=  epsilon
        C_minus, _ = get_cost(theta_minus, ee_position)
        
        # # Restoring the original joint angle
        # joint_angles[i] += epsilon
        
        # Compute the numerical gradient using central difference
        gradient[i] = (C_plus - C_minus) / (2 * epsilon)
    
    return gradient


# def inverse_kinematics_with_gradient(target_position):
#     joint_angles = np.array([0.0, 0.0, 0.0])  # Initial guess for joint angles
#     alpha = 0.54
#     max_iterations = 30000
#     tolerance = 0.0001
#     sq_error = 100

#     iteration = 0
#     while (iteration < max_iterations and sq_error >tolerance):
#         gradient = get_gradient(joint_angles, target_position)
#         #going in the direction of negative gradient 
#         joint_angles = joint_angles -  (alpha*gradient) # updating joint angles 
#         sq_err, mean_err = get_cost(joint_angles, target_position)  # Get mean error for convergence check
#         iteration+=1 
#     return joint_angles



def inverse_kinematics_with_gradient(target_position):
    joint_angles = np.array([0.0, 0.0, 0.0])  # Initial guess for joint angles
    alpha = 0.54
    max_iterations = 30000
    tolerance = 0.00001
    mean_error = 100

    iteration = 0
    while (iteration < max_iterations and mean_error >tolerance):
        gradient = get_gradient(joint_angles, target_position)
        #going in the direction of negative gradient 
        joint_angles = joint_angles -  (alpha*gradient) # updating joint angles 
        sq_err, mean_error = get_cost(joint_angles, target_position)  # Get mean error for convergence check
        iteration+=1 
    return joint_angles
