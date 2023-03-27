# Same as v5, now with yaw pitch roll from both the stabilizer (gyro) and our own lighthouse angles 
# graphed and orientation matrix also graphed. Notes: The yaw and pitch measurements match up pretty well, 
# but our roll measurements seem to have some trouble when rolling too far away from the 
# lighthouse (facing the opposite direction), which is expected.
import logging
import time
import numpy as np
from threading import Event as Ev
from arena import *
from multiprocessing import Process,Queue,Pipe
from m1 import f
from m1 import test

from cflib.crazyflie.mem import LighthouseMemHelper
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from cflib.crazyflie.log import LogConfig

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E7E7'

global bs0_rot_mat
global bs0_origin

global bs1_rot_mat
global bs1_origin

parent_conn,child_conn = Pipe()

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


def log_pos_callback(timestamp, data, logconf):
    pos_x = data['lighthouse.x']
    pos_y = data['lighthouse.y']
    pos_z = data['lighthouse.z']


   # print('pos: ({}, {}, {})'.format(pos_x, pos_y, pos_z))
    child_conn.send((pos_x,pos_y,pos_z))

def log_stab_callback(timestamp, data, logconf):
    stab_y = data['stabilizer.yaw']
    stab_p = data['stabilizer.pitch']
    stab_r = data['stabilizer.roll']

    #print('stab: ({}, {}, {})'.format(stab_y, stab_p, stab_r))
    child_conn.send(("stab",stab_y,stab_p,stab_r))

def simple_log_async(scf, logconfs):
    cf = scf.cf
    for conf in logconfs:
        cf.log.add_config(conf)
    logconfs[0].data_received_cb.add_callback(log_pos_callback)
    logconfs[1].data_received_cb.add_callback(log_stab_callback)
    for conf in logconfs:
        conf.start()
    time.sleep(100) # how long to collect data for
    for conf in logconfs:
        conf.stop()

class ReadMem:
    def __init__(self, uri):
        self._event = Ev()

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
            elif (id == 1):
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

def main():
    # Initialize the low-level drivers
    print("what")
    cflib.crtp.init_drivers()

    # arenaxr process
    p = Process(target=f, args=(parent_conn,))
    p.start()

    # ground truth
    lg_pos = LogConfig(name='position', period_in_ms=50)
    lg_pos.add_variable('lighthouse.x', 'float')
    lg_pos.add_variable('lighthouse.y', 'float')
    lg_pos.add_variable('lighthouse.z', 'float')

    lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')

    ReadMem(uri)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        # set method to cross-beam method to get lighthouse.x,y,z
        cf = scf.cf
        param_name = "lighthouse.method" 
        param_value = 0 # Estimation Method: 0:CrossingBeam, 1:Sweep in EKF (extended Kalman Filter) (default: 1)
        cf.param.set_value(param_name, param_value)
        
        simple_log_async(scf, [lg_stab, lg_pos])

if __name__ == '__main__':
    main()