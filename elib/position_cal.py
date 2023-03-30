# Helper functions for calculating the position and orientation from the raw sweep angles.

import numpy as np
import math
# https://github.com/bitcraze/crazyflie-firmware/blob/a70c357a7c03b84a0caebe6d31a5fb0ccc40ff2b/src/utils/src/lighthouse/lighthouse_geometry.c
# https://github.com/bitcraze/crazyflie-firmware/blob/2020.06/src/utils/src/lighthouse/lighthouse_geometry.c#L63-L102
# https://github.com/ashtuchkin/vive-diy-position-sensor/wiki/Position-calculation-in-detail
# https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/lighthouse/angle_conversion/

# helper function to get the normal vector to the horizontal and vertical planes
def getV(hor, vert):
    v_hor = np.array([math.sin(hor),-math.cos(hor),0])
    v_vert = np.array([-math.sin(vert), 0, math.cos(vert)])
    return v_hor, v_vert

# given a lists of four angles from the same sensor, return global rays that intersect the lighthouse and the sensor
def parseData(data, bs0_rot_mat, bs1_rot_mat): 
    # print(data,bs0_rot_mat,bs1_rot_mat)
    bs0_angle_hor, bs0_angle_vert, bs1_angle_hor, bs1_angle_vert = data

    # get the normal vectors to the lighthouse sweep planes
    bs0_v_hor, bs0_v_vert = getV(bs0_angle_hor, bs0_angle_vert)
    bs1_v_hor, bs1_v_vert = getV(bs1_angle_hor, bs1_angle_vert)

    # get the intersection of the vertical and horizontal planes to get the ray from the lighthouse to the sensor in the lighthouse reference frame
    bs0_u = np.cross(bs0_v_vert, bs0_v_hor)
    bs1_u = np.cross(bs1_v_vert, bs1_v_hor)

    bs0_u = bs0_u/np.linalg.norm(bs0_u)
    bs1_u = bs1_u/np.linalg.norm(bs1_u)

    # convert the rays to the global reference frame
    u0_glob = bs0_rot_mat@bs0_u
    u1_glob = bs1_rot_mat@bs1_u

    return u0_glob, u1_glob

# given two lines, find the position of the intersection
def intersectLines(input, bs0_origin, bs1_origin):
    line0, line1 = input
    w_0 = bs0_origin - bs1_origin
    a = np.dot(line0,line0)
    b = np.dot(line0,line1)
    c = np.dot(line1,line1)
    d = np.dot(line0,w_0)
    e = np.dot(line1,w_0)
    denom = a*c - b*b
    # print(line1)
    # print(a,b,c,denom)
    if(denom != 0):
        s = (b*e - c*d) / denom
        t = (a*e - b*d) / denom
    ps = bs0_origin + s*line0
    qt = bs1_origin + t*line1
    mid = (ps+qt)/2
    return mid

# give three sensor positions, find the x,y,z vectors as a matrix
def get_orient(three_pos):
    sensor_pos0, sensor_pos1, sensor_pos2 = three_pos
    x_vec = sensor_pos0-sensor_pos2
    y_vec = sensor_pos1-sensor_pos0
    z_vec = np.cross(x_vec,y_vec)
    return np.array([x_vec/np.linalg.norm(x_vec),y_vec/np.linalg.norm(y_vec),z_vec/np.linalg.norm(z_vec)])

def getYPR(orient):
    x_vec, y_vec, z_vec = orient
    #print(orient)
    yaw = (math.atan2(-x_vec[1],-x_vec[0]))*(180/math.pi)
    pitch = (math.atan2(-x_vec[2],math.sqrt(y_vec[2]**2+z_vec[2]**2)))*(180/math.pi)
    roll = -(math.atan2(y_vec[2],z_vec[2]))*(180/math.pi)
    #print(yaw,pitch,roll)
    return yaw, pitch, roll