# Same as v5, now with yaw pitch roll from both the stabilizer (gyro) and our own lighthouse angles 
# graphed and orientation matrix also graphed. Notes: The yaw and pitch measurements match up pretty well, 
# but our roll measurements seem to have some trouble when rolling too far away from the 
# lighthouse (facing the opposite direction), which is expected.
import logging
import time
import numpy as np
from threading import Event as Ev
from arena import *
from multiprocessing import Process,Pipe
from arenaxr_rtpos import f

from cflib.crazyflie.mem import LighthouseMemHelper
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
    logconfs[1].data_received_cb.add_callback(log_pos_callback)
    for conf in logconfs:
        conf.start()
    time.sleep(100) # how long to collect data for
    for conf in logconfs:
        conf.stop()

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

    lg_stab = LogConfig(name='Stabilizer', period_in_ms=50)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        # set method to cross-beam method to get lighthouse.x,y,z
        cf = scf.cf
        param_name = "lighthouse.method" 
        param_value = 0 # Estimation Method: 0:CrossingBeam, 1:Sweep in EKF (extended Kalman Filter) (default: 1)
        cf.param.set_value(param_name, param_value)
        
        simple_log_async(scf, [lg_stab, lg_pos])

if __name__ == '__main__':
    main()