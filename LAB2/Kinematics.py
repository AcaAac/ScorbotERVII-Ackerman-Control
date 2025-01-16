import math as m
import numpy as np

def normalizeAngle(angle):
    """
    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle 

def kinematic_model(path_x, path_y, GPS_x, GPS_y):
        
    """
    :param x:                   (float vector) path's x-coordinates [m]
    :param y:                   (float vector) path's y-coordinates [m]
    :param GPS_x:               (float vector (?) ) vehicle's x-coordinate [m]
    :param GPS_y:               (float vector (?) ) vehicle's y-coordinate [m]

    :return ws:                 (float) vehicle's steering angle speed [rad/s]
    """

    dt = 1                     # Discrete time period [s]
    L = 2.46                   # Vehicle's axis distance [m]
    max_phi = m.radians(30)    # Vehicle's steering limits [rad]  

    theta = [0]*len(path_x)
    velocity = [0]*len(path_x)
    phi = [0]*len(theta)

    for i in range(len(path_x)-1):
        theta[i] = m.atan2(path_y[i+1]-path_y[i],path_x[i+1]-path_x[i])
        velocity[i] = m.sqrt((path_x[i+1]-path_x[i])**2 + (path_y[i+1]-path_y[i])**2)/dt
    theta[-1] = m.atan2(path_y[1]-path_y[-1],path_x[1]-path_x[-1])
    velocity[-1] = m.sqrt((path_x[1]-path_x[-1])**2 + (path_y[1]-path_y[-1])**2)/dt

    theta_deg = [m.degrees(x) for x in theta]
    print("theta",theta_deg)
    print("velocity",velocity)

    for i in range(len(theta)-1):
        theta[i] = normalizeAngle(theta[i])
        phi[i] = m.atan2((normalizeAngle(theta[i+1]-theta[i]))/dt * L, velocity[i])
    phi[-1] = m.atan2((theta[1]-theta[-1]) * L, velocity[-1])

    phi_deg = [m.degrees(x) for x in phi]
    print("phi",phi_deg)

    for i in range(len(phi)):
        phi[i] = normalizeAngle(phi[i])
        phi[i] = -max_phi if phi[i] < -max_phi else max_phi if phi[i] > max_phi else phi[i]  
    norm_phi_deg = [m.degrees(x) for x in phi]
    print("normalized phi",norm_phi_deg)

    return phi # NOT phi , it should be ws


x = [2, 3, 4, 4, 3, 2, 1, 1, 2]
y = [1, 1, 2, 3, 4, 4, 3, 2, 1]


actual_x = [2.1, 2,9, 4, 4.2, 3.1, 2, 0.9, 1, 2.1]
actual_y = [1, 1.1, 2.1, 3, 3.9, 3.8, 3.1, 1.8, 1.1]

ws = kinematic_model(x,y,actual_x,actual_y)

print(ws)