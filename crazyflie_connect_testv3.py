from audioop import avg
from curses import raw
import logging
import math
import time
import numpy as np
from threading import Event

from cflib.crazyflie.mem import LighthouseMemHelper
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt

import position_cal as pc
import pos_grapher as pg


# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E7E7'

global bs0_rot_mat
global bs0_origin

global bs1_rot_mat
global bs1_origin

global prev_pos
global curr_pos
global valid_sensors

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# log the yaw pitch roll angles to the ypr_file
def log_stab_callback(timestamp, data, logconf):
    ypr_file.write('[%d][%s]: %s \n' % (timestamp, logconf.name, data))

# log the raw angles to the raw_angles_file
# calculate the position of the crazyflie from the raw angles of the sensors
# log the calculated position of the crazyflie to the my_pos_file
# add the calculated postion to the list of calculated positions for graphing
def log_raw_callback(timestamp, data, logconf): 
    sensor_num = int(logconf.name[-1]) # get the sensor index/number from the log name

    raw_angles_file.write('[%d][%s]: %s \n' % (timestamp, logconf.name, data))
    angles = []
    for key in data:
        angles.append(data[key])
    pos = pc.intersectLines(pc.parseData(angles,bs0_rot_mat,bs1_rot_mat),bs0_origin,bs1_origin)

    global prev_pos
    global curr_pos
    global valid_sensors

    # check to see if the position is valid
    if not np.array_equal(pos,prev_pos[sensor_num]):
        valid_sensors[sensor_num] = 1
    else:
        valid_sensors[sensor_num] = 0

    curr_pos[sensor_num] = pos
    
    # if this is the last sensor, calculate the average position from all four sensors if valid
    if sensor_num == 3:
        avg_pos = [0,0,0]
        for i in range(0,len(curr_pos)):
            ar = curr_pos[i]
            if valid_sensors[i]:
                avg_pos = np.add(avg_pos,ar)
        # only calculate the position if all four sensors are valid
        tot_valid = np.sum(valid_sensors) 
        if(tot_valid == 4):
            avg_pos = np.divide(avg_pos,tot_valid)
            global my_pos_coords
            print(avg_pos)
            my_pos_coords.append(avg_pos)
            my_pos_file.write('%s %s %s\n' % (avg_pos[0],avg_pos[1],avg_pos[2]))
        valid_sensors = [0,0,0,0]
        prev_pos = curr_pos

# log the position of the crazyflie to the pos_file
# add the postion to the list of true positions for graphing
def log_pos_callback(timestamp, data, logconf):
    pos_x = data['lighthouse.x']
    pos_y = data['lighthouse.y']
    pos_z = data['lighthouse.z']
    global their_pos_coords
    their_pos_coords.append((pos_x,pos_y,pos_z))
    pos_file.write('%s %s %s\n' % (pos_x,pos_y,pos_z))

def simple_log_async(scf, logconfs):
    cf = scf.cf
    for conf in logconfs:
        cf.log.add_config(conf)
    logconfs[0].data_received_cb.add_callback(log_stab_callback)
    logconfs[1].data_received_cb.add_callback(log_pos_callback)
    for i in range(2,len(logconfs)):
        logconfs[i].data_received_cb.add_callback(log_raw_callback)
    for conf in logconfs:
        conf.start()
    time.sleep(10)
    for conf in logconfs:
        conf.stop()


# read the memory from the Crazyflie to find the origin and rotation matrices of the lighthouses
# https://github.com/bitcraze/crazyflie-lib-python/blob/master/examples/lighthouse/read_lighthouse_mem.py
class ReadMem:
    def __init__(self, uri):
        self._event = Event()

        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            helper = LighthouseMemHelper(scf.cf)

            helper.read_all_geos(self._geo_read_ready)
            self._event.wait()

            self._event.clear()

            helper.read_all_calibs(self._calib_read_ready)
            self._event.wait()

    def _geo_read_ready(self, geo_data):
        for id, data in geo_data.items():
            print('---- Geometry for base station', id + 1)
            data.dump()
            if(id == 0):
                global bs0_origin
                bs0_origin = np.array(data.origin)
                global bs0_rot_mat
                bs0_rot_mat = np.array(data.rotation_matrix)
            else:
                global bs1_origin
                bs1_origin = np.array(data.origin)
                global bs1_rot_mat
                bs1_rot_mat = np.array(data.rotation_matrix)
            print()
        self._event.set()

    def _calib_read_ready(self, calib_data):
        for id, data in calib_data.items():
            print('---- Calibration data for base station', id + 1)
            data.dump()
            print()
        self._event.set()


if __name__ == '__main__':
    # plotting
    map = plt.figure()
    ax1 = map.add_subplot(2, 2, 1, projection='3d')
    ax2 = map.add_subplot(2, 2, 2, projection='3d')
    map_ax = map.add_subplot(2, 2, 3, projection='3d')

    map_ax.autoscale(enable=True, axis='both', tight=True)
    ax1.autoscale(enable=True, axis='both', tight=True)
    ax2.autoscale(enable=True, axis='both', tight=True)

    # # # Setting the axes properties
    map_ax.set_xlim3d([-0.5, 1])
    map_ax.set_ylim3d([-0.5, 1])
    map_ax.set_zlim3d([-0.5, 1])

    ax1.set_xlim3d([-0.5, 1])
    ax1.set_ylim3d([-0.5, 1])
    ax1.set_zlim3d([-0.5, 1])

    ax2.set_xlim3d([-0.5, 1])
    ax2.set_ylim3d([-0.5, 1])
    ax2.set_zlim3d([-0.5, 1])

    hl, = map_ax.plot3D([0], [0], [0])
    hb, = map_ax.plot3D([0], [0], [0])

    h1, = ax1.plot3D([0], [0], [0])
    h2, = ax2.plot3D([0], [0], [0], color = "orange")

    # arrays containing the calculated position and the ground truth position for plotting after logging ends
    my_pos_coords = []
    their_pos_coords = []

    #raw_angles file
    raw_angles_file = open("raw_angles.txt","a")
    raw_angles_file.truncate(0)

    #my position file
    my_pos_file = open("my_pos_file.txt","a")
    my_pos_file.truncate(0)

    #yaw-pitch-row file
    ypr_file = open("yaw_pitch_row.txt","a")
    ypr_file.truncate(0)

    #position file
    pos_file = open("position.txt","a")
    pos_file.truncate(0)

    # initialize arrays for position calculation
    prev_pos = [[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
    curr_pos = [[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
    valid_sensors = [0,0,0,0]

    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    # ground truth
    lg_stab = LogConfig(name='Stabilizer', period_in_ms=100)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')

    lg_pos = LogConfig(name='position', period_in_ms=100)
    lg_pos.add_variable('lighthouse.x', 'float')
    lg_pos.add_variable('lighthouse.y', 'float')
    lg_pos.add_variable('lighthouse.z', 'float')

    # calibrated lh1 angles
    lg_lh_sen0 = LogConfig(name='lighthouse0', period_in_ms=10)
    lg_lh_sen0.add_variable('lighthouse.angle0x', 'float')
    lg_lh_sen0.add_variable('lighthouse.angle0y', 'float')
    lg_lh_sen0.add_variable('lighthouse.angle1x', 'float')
    lg_lh_sen0.add_variable('lighthouse.angle1y', 'float')

    lg_lh_sen1 = LogConfig(name='lighthouse1', period_in_ms=10)
    lg_lh_sen1.add_variable('lighthouse.angle0x_1', 'float')
    lg_lh_sen1.add_variable('lighthouse.angle0y_1', 'float')
    lg_lh_sen1.add_variable('lighthouse.angle1x_1', 'float')
    lg_lh_sen1.add_variable('lighthouse.angle1y_1', 'float')

    lg_lh_sen2 = LogConfig(name='lighthouse2', period_in_ms=10)
    lg_lh_sen2.add_variable('lighthouse.angle0x_2', 'float')
    lg_lh_sen2.add_variable('lighthouse.angle0y_2', 'float')
    lg_lh_sen2.add_variable('lighthouse.angle1x_2', 'float')
    lg_lh_sen2.add_variable('lighthouse.angle1y_2', 'float')

    lg_lh_sen3 = LogConfig(name='lighthouse3', period_in_ms=10)
    lg_lh_sen3.add_variable('lighthouse.angle0x_3', 'float')
    lg_lh_sen3.add_variable('lighthouse.angle0y_3', 'float')
    lg_lh_sen3.add_variable('lighthouse.angle1x_3', 'float')
    lg_lh_sen3.add_variable('lighthouse.angle1y_3', 'float')

    ReadMem(uri)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        # set method to cross-beam method to get lighthouse.x,y,z
        cf = scf.cf
        param_name = "lighthouse.method"
        param_value = 0
        cf.param.set_value(param_name, param_value)
        
        log_confs = [lg_stab, lg_pos, lg_lh_sen0, lg_lh_sen1, lg_lh_sen2, lg_lh_sen3]
        simple_log_async(scf, log_confs)

    # graph the crazyflie's position
    for i in my_pos_coords: # our calculated position
        pg.update_line(hl, list(i))
        pg.update_line(h1, list(i))
        
        plt.show(block=False)

    for i in their_pos_coords: # their calculated position
        pg.update_line(hb, list(i))
        pg.update_line(h2, list(i))
        plt.show(block=False)

    plt.show()

    raw_angles_file.close()
    my_pos_file.close()
    ypr_file.close()
    pos_file.close()

    


