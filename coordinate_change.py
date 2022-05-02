import numpy as np



def translation(vector):
    return np.array([[1, 0, 0, vector[0]],
                     [0, 1, 0, vector[1]],
                     [0, 0, 1, vector[2]],
                     [0, 0, 0, 1]])


def Rx(alpha):
    return np.array([[1, 0, 0, 0],
                     [0, np.cos(alpha), -np.sin(alpha), 0],
                     [0, np.sin(alpha), np.cos(alpha), 0],
                     [0, 0, 0, 1]])

def Ry(alpha):
    return np.array([[np.cos(alpha), 0, np.sin(alpha), 0],
                     [0, 1, 0, 0],
                     [-np.sin(alpha), 0, np.cos(alpha), 0],
                     [0, 0, 0, 1]])
        
def Rz(alpha):
    return np.array([[np.cos(alpha), -np.sin(alpha), 0, 0],
                     [np.sin(alpha), np.cos(alpha), 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])


def transform(T, vector):
    vector = np.array(vector)
    vector = np.swapaxes([vector], 0, 1)
    vector = np.vstack((vector, [1]))
    output = (T @ vector)[:3]
    return output

'''returns the point in the new coordinate systeme with x rotation'''
def change_coordinate_x(angle,trans_vector,point):
    trans = translation(np.array(trans_vector))
    rot = Rz(angle)
    point_n = trans @ rot @ [*point, 1]
    return  np.array([point_n[0],point_n[1],point_n[2]])



'''Faster way of computing the inverse of a 4x4 homogeneous matrix'''
def frame_inv(T):
    R = T[:3, :3] # On extrait la rotation
    t = T[:3, 3:] # On extrait la translation
    upper = np.hstack((R.T, -R.T @ t))
    lower = np.array([0., 0., 0., 1.])
    return np.vstack((upper, lower))
