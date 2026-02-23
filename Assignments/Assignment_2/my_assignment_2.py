import math
import numpy as np

# new code --------------------
# helper functions 

def rotation_x(theta):
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    return np.array([
        [1, 0, 0, 0],
        [0, cos_t, -sin_t, 0],
        [0, sin_t, cos_t, 0],
        [0, 0, 0, 1]
    ])

def rotation_y(theta):
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    return np.array([
        [cos_t, 0, sin_t, 0],
        [0, 1, 0, 0],
        [-sin_t, 0, cos_t, 0],
        [0, 0, 0, 1]
    ])

def rotation_z(theta):
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    return np.array([
        [cos_t, -sin_t, 0, 0],
        [sin_t, cos_t, 0, 0],
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


# no frame change or translation
# just rotation around z axis
def get_T01(theta_1):
    return rotation_z(theta_1)
   

# rotate about z by theta_2 for frame 
# then rotate about x by 90 degree to flip y and z axes
# tranlate along y by 0.3 (f2 wrt to f1)
# angle as seen from parent to child frame
def get_T12(theta_2):
    return translation(0,0.3,0) @ rotation_x(math.pi/2) @ rotation_z(theta_2)
    

   
#rotation around z axis
#offset along x
#no change in frame orientation
def get_T23(theta_3):
    # frame 3 + 0.4 wrt to frame 2 along x 
    return  translation(0.4,0,0) @ rotation_z(theta_3) 
    

#orientation change : -90 degree rotation about x 
#frame 4 is attached to joint 3, no rotation, only upward translation along y wrt to frame 3
def get_T34():
    return  translation(0,0.3,0) @ rotation_x(-math.pi/2) 

# angle as seen from parent to child frame

def get_FK(theta_1, theta_2, theta_3):
    return get_T01(theta_1) @ get_T12(theta_2) @ get_T23(theta_3) @ get_T34()



#--------------- Part B ------------------
def ee_in_collision(angles, p_point, tolerance):
    theta_1, theta_2, theta_3 = angles
    T_0_ee = get_FK(theta_1, theta_2, theta_3) 
    ee_pos = (T_0_ee @ np.array([0,0,0,1]).reshape(4,1))[0:3,0] #taking first 3 elements
    if np.linalg.norm(ee_pos - p_point) < tolerance:
        return True
    return False 


#--------------- Part C ------------------

def path_in_collision(path, object_list):
    for obj in object_list : 
        pos = obj[0] #3x1 numpy array
        object_radius = obj[1] #scalar radius 

        for angle in path: 
            if ee_in_collision(angle,pos,object_radius):
                return True #break if any one of the objects create a collison with the robot
            else: 
                continue 
    return False

