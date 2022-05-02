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

import pos_grapher as pg

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E7E7'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


def log_pos_callback(timestamp, data, logconf):
    positions.append([data['stateEstimate.x'],data['stateEstimate.y'],data['stateEstimate.z']])

if __name__ == '__main__':
    
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        param_name = "lighthouse.method"
        param_value = 1 #0 for cross-beam, 1 for sweep (kalman)
        cf.param.set_value(param_name, param_value)

    map = plt.figure()
    map_ax = Axes3D(map)
    map_ax.autoscale(enable=True, axis='both', tight=True)
   
    # # # Setting the axes properties
    map_ax.set_xlim3d([-0.5, 1])
    map_ax.set_ylim3d([-0.5, 1])
    map_ax.set_zlim3d([-0.5, 1])

    hl, = map_ax.plot3D([0], [0], [0])

    positions = []

    cflib.crtp.init_drivers()

    lg_stateEsti = LogConfig(name='StateEstimator', period_in_ms=100)
    lg_stateEsti.add_variable('stateEstimate.roll', 'float')
    lg_stateEsti.add_variable('stateEstimate.pitch', 'float')
    lg_stateEsti.add_variable('stateEstimate.yaw', 'float')


    for pos in positions:
        pg.update_line(map,list(pos))

    plt.show()