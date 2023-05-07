
import logging
import time
import numpy as np
from threading import Event as Ev
from arena import *
from multiprocessing import Process,Pipe
from arenaxr_rtpos import f

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from cflib.crazyflie.log import LogConfig

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E7E7'

parent_conn, child_conn = Pipe()

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


def log_pos_callback(timestamp, data, logconf):
    pos_x = data['lighthouse.x']
    pos_y = data['lighthouse.y']
    pos_z = data['lighthouse.z']

    child_conn.send(("pos",pos_x,pos_y,pos_z))

# kalman callback
def log_kpos_callback(timestamp, data, logconf):
    pos_x = data['stateEstimate.x']
    pos_y = data['stateEstimate.y']
    pos_z = data['stateEstimate.z']

    k_yaw = data['stateEstimate.yaw']
    k_pitch = data['stateEstimate.pitch']
    k_roll = data['stateEstimate.roll']

    child_conn.send(("pos",pos_x,pos_y,pos_z))
    # child_conn.send(("stab",k_yaw,k_pitch,k_roll))

def log_stab_callback(timestamp, data, logconf):
    stab_y = data['stabilizer.yaw']
    stab_p = data['stabilizer.pitch']
    stab_r = data['stabilizer.roll']

    child_conn.send(("stab",stab_y,stab_p,stab_r))

def simple_log_async(scf, logconfs):
    cf = scf.cf
    for conf in logconfs:
        cf.log.add_config(conf)
    logconfs[0].data_received_cb.add_callback(log_stab_callback)
    # logconfs[1].data_received_cb.add_callback(log_pos_callback)
    logconfs[2].data_received_cb.add_callback(log_kpos_callback)
    for conf in logconfs:
        conf.start()
    time.sleep(100) # how long to collect data for
    for conf in logconfs:
        conf.stop()

def main():
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    # arenaxr process
    p = Process(target=f, args=(parent_conn,))
    p.start()

    # ground truth
    lg_pos = LogConfig(name='position', period_in_ms=100)
    lg_pos.add_variable('lighthouse.x', 'float')
    lg_pos.add_variable('lighthouse.y', 'float')
    lg_pos.add_variable('lighthouse.z', 'float')

    lg_stab = LogConfig(name='Stabilizer', period_in_ms=50)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')

    lg_kpos = LogConfig(name='Kalman', period_in_ms=50)
    lg_kpos.add_variable('stateEstimate.x', 'float')
    lg_kpos.add_variable('stateEstimate.y', 'float')
    lg_kpos.add_variable('stateEstimate.z', 'float')
    lg_kpos.add_variable('stateEstimate.roll', 'float')
    lg_kpos.add_variable('stateEstimate.pitch', 'float')
    lg_kpos.add_variable('stateEstimate.yaw', 'float')

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        # set method to cross-beam method to get lighthouse.x,y,z instead of StateEstimate.x,y,z
        cf = scf.cf
        param_name = "lighthouse.method" 
        param_value = 1 # Estimation Method: 0:CrossingBeam, 1:Sweep in EKF (extended Kalman Filter) (default: 1)
        cf.param.set_value(param_name, param_value)
        
        simple_log_async(scf, [lg_stab, lg_pos, lg_kpos])

if __name__ == '__main__':
    main()