# Same as v3 except the log blocks are organized by angle (four same angles from all four sensors come in 
# on the same timestamp Ex. The horizontal angle from Base Station 1). Does not take into account whether 
# or not sensors are blocked.

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

# v4: change the loggers to sync angles instead of sensors


# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E7E7'

global bs0_rot_mat
global bs0_origin

global bs1_rot_mat
global bs1_origin

global prev_pos
global curr_pos

sensor_angles = [[],[],[],[]]
sensor_pos = []

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def log_stab_callback(timestamp, data, logconf):
    
    ypr_file.write('[%d][%s]: %s \n' % (timestamp, logconf.name, data))
    #print('[%d][%s]: %s' % (timestamp, logconf.name, data))

def log_raw_callback(timestamp, data, logconf):
    global sensor_angles
    global sensor_pos
    raw_angles_file.write('[%d][%s]: %s \n' % (timestamp, logconf.name, data))
    for count, key in enumerate(data):
        sensor_angles[count].append(data[key])
        print(data[key])
        if(len(sensor_angles[count]) == 4):
            pos = pc.intersectLines(pc.parseData(sensor_angles[count],bs0_rot_mat,bs1_rot_mat),bs0_origin,bs1_origin)
            sensor_angles[count] = []
            sensor_pos.append(pos)
            if(len(sensor_pos) == 4):
                my_pos_coords.append(np.divide(np.sum(sensor_pos,axis=0),4))
                sensor_pos = []

def log_pos_callback(timestamp, data, logconf):
    #pos_file.write('[%d][%s]: %s \n' % (timestamp, logconf.name, data))
    pos_x = data['lighthouse.x']
    pos_y = data['lighthouse.y']
    pos_z = data['lighthouse.z']
    #print(pos_x,pos_y,pos_z)
    global their_pos_coords
    their_pos_coords.append((pos_x,pos_y,pos_z))
    pos_file.write('%s %s %s\n' % (pos_x,pos_y,pos_z))
    #print('[%d][%s]: %s' % (timestamp, logconf.name, data))

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

    prev_pos = [[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
    curr_pos = [[0,0,0],[0,0,0],[0,0,0],[0,0,0]]

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
    lg_lh_sen0.add_variable('lighthouse.angle0x_1', 'float')
    lg_lh_sen0.add_variable('lighthouse.angle0x_2', 'float')
    lg_lh_sen0.add_variable('lighthouse.angle0x_3', 'float')

    lg_lh_sen1 = LogConfig(name='lighthouse1', period_in_ms=10)
    lg_lh_sen1.add_variable('lighthouse.angle0y', 'float')
    lg_lh_sen1.add_variable('lighthouse.angle0y_1', 'float')
    lg_lh_sen1.add_variable('lighthouse.angle0y_2', 'float')
    lg_lh_sen1.add_variable('lighthouse.angle0y_3', 'float')

    lg_lh_sen2 = LogConfig(name='lighthouse2', period_in_ms=10)
    lg_lh_sen2.add_variable('lighthouse.angle1x', 'float')
    lg_lh_sen2.add_variable('lighthouse.angle1x_1', 'float')
    lg_lh_sen2.add_variable('lighthouse.angle1x_2', 'float')
    lg_lh_sen2.add_variable('lighthouse.angle1x_3', 'float')

    lg_lh_sen3 = LogConfig(name='lighthouse3', period_in_ms=10)
    lg_lh_sen3.add_variable('lighthouse.angle1y', 'float')
    lg_lh_sen3.add_variable('lighthouse.angle1y_1', 'float')
    lg_lh_sen3.add_variable('lighthouse.angle1y_2', 'float')
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

    for i in my_pos_coords:
        pg.update_line(hl, list(i))
        pg.update_line(h1, list(i))
        
        plt.show(block=False)

    for i in their_pos_coords:
        pg.update_line(hb, list(i))
        pg.update_line(h2, list(i))
        plt.show(block=False)

    plt.show()

    raw_angles_file.close()
    my_pos_file.close()
    ypr_file.close()
    pos_file.close()

    


