# Lighthouse position calculations from raw angles compared to logged xyz calculated by Crazyflie firmware. 
# The orange line is ground truth from Crazyflie firmware and the blue line is our line generated from our 
# position calculation algorithm. Uses Lighthouse 2 angles, which is not correct. Also only finds the position of one sensor.
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


# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E7E7'

global bs0_rot_mat
global bs0_origin

global bs1_rot_mat
global bs1_origin
global my_pos_coords
global their_pos_coords

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def makeRot(angle):
    return np.array([[math.cos(angle),-math.sin(angle),0],[math.sin(angle),math.cos(angle),0],[0,0,1]])

def getV(hor, vert):
    r1 = makeRot(hor)
    r2 = makeRot(vert)
    t = math.pi/6
    #t = 0
    v_hor = r1@np.array([0,-math.cos(t),math.sin(t)])
    v_vert = r2@np.array([0,-math.cos(t),-math.sin(t)])
    return v_hor, v_vert

# def getV(hor, vert):
#     t = math.pi/6
#     hor1 = (hor + vert) / 2.0
#     beta = vert - hor
#     vert1 = math.atan(math.sin(beta) / math.tan(t)*(math.cos(hor)+math.cos(vert)))
    
#     r1 = makeRot(hor1)
#     r2 = makeRot(vert1)
#     v_hor1 = r1@np.array([0,-math.cos(math.pi/6),math.sin(math.pi/6)])
#     v_vert1 = r2@np.array([0,-math.cos(math.pi/6),-math.sin(math.pi/6)])
#     return v_hor1, v_vert1



def parseData(data):
    bs0_angle_hor = data['lighthouse.angle0x_0lh2']
    bs0_angle_vert = data['lighthouse.angle0y_0lh2']
    bs1_angle_hor = data['lighthouse.angle1x_0lh2']
    bs1_angle_vert = data['lighthouse.angle1y_0lh2']
    # bs0_angle_hor = data['lighthouse.angle0x']
    # bs0_angle_vert = data['lighthouse.angle0y']
    # bs1_angle_hor = data['lighthouse.angle1x']
    # bs1_angle_vert = data['lighthouse.angle1y']

    bs0_v_hor, bs0_v_vert = getV(bs0_angle_hor, bs0_angle_vert)
    bs1_v_hor, bs1_v_vert = getV(bs1_angle_hor, bs1_angle_vert)

    bs0_u = np.cross(bs0_v_hor, bs0_v_vert)
    bs1_u = np.cross(bs1_v_hor, bs1_v_vert)

    u0_glob = bs0_rot_mat@bs0_u + bs0_origin
    u1_glob = bs1_rot_mat@bs1_u + bs1_origin

    return u0_glob, u1_glob

def intersectLines(input):
    line0, line1 = input
    w_0 = bs0_origin - bs1_origin
    a = np.dot(line0,line0)
    b = np.dot(line0,line1)
    c = np.dot(line1,line1)
    d = np.dot(line0,w_0)
    e = np.dot(line1,w_0)
    denom = a*c - b*b
    if(denom != 0):
        s = (b*e - c*d) / denom
        t = (a*e - b*d) / denom
    ps = bs0_origin + s*line0
    qt = bs1_origin + t*line1
    mid = (ps+qt)/2
    mid[2] = -mid[2]
    mid[0] = -mid[0]
    mid[1] = -mid[1]
    return mid


def log_stab_callback(timestamp, data, logconf):
    
    ypr_file.write('[%d][%s]: %s \n' % (timestamp, logconf.name, data))
    #print('[%d][%s]: %s' % (timestamp, logconf.name, data))

def log_raw_callback(timestamp, data, logconf):
    
    raw_angles_file.write('[%d][%s]: %s \n' % (timestamp, logconf.name, data))
    pos = intersectLines(parseData(data))
    print(pos)
    global my_pos_coords
    my_pos_coords.append(pos)
    #print('[%d][%s]: %s' % (timestamp, logconf.name, data))

def log_pos_callback(timestamp, data, logconf):
    
    pos_file.write('[%d][%s]: %s \n' % (timestamp, logconf.name, data))
    pos_x = data['lighthouse.x']
    pos_y = data['lighthouse.y']
    pos_z = data['lighthouse.z']
    global their_pos_coords
    their_pos_coords.append((pos_x,pos_y,pos_z))
    #print('[%d][%s]: %s' % (timestamp, logconf.name, data))

def simple_log_async(scf, logconf1, logconf2, logconf3):
    cf = scf.cf
    cf.log.add_config(logconf1)
    cf.log.add_config(logconf2)
    cf.log.add_config(logconf3)
    logconf1.data_received_cb.add_callback(log_stab_callback)
    logconf2.data_received_cb.add_callback(log_raw_callback)
    logconf3.data_received_cb.add_callback(log_pos_callback)
    logconf1.start()
    logconf2.start()
    logconf3.start()
    time.sleep(10)
    logconf1.stop()
    logconf2.stop()
    logconf3.stop()


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
            #print(data.origin)
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

def update_line(hl, new_data):
	xdata, ydata, zdata = hl._verts3d
	hl.set_xdata(np.append(xdata, new_data[0]))
	hl.set_ydata(np.append(ydata, new_data[1]))
	hl.set_3d_properties(np.append(zdata, new_data[2]))
	plt.draw()



if __name__ == '__main__':
    # plotting
    map = plt.figure()
    map_ax = Axes3D(map)
    map_ax.autoscale(enable=True, axis='both', tight=True)

    # # # Setting the axes properties
    map_ax.set_xlim3d([-0.5, 1])
    map_ax.set_ylim3d([-0.5, 1])
    map_ax.set_zlim3d([-0.5, 1])

    hl, = map_ax.plot3D([0], [0], [0])
    hb, = map_ax.plot3D([0], [0], [0])

    global my_pos_coords
    my_pos_coords = []

    global their_pos_coords
    their_pos_coords = []

    #raw_angles file
    raw_angles_file = open("raw_angles.txt","a")
    raw_angles_file.truncate(0)

    #yaw-pitch-row file
    ypr_file = open("yaw_pitch_row.txt","a")
    ypr_file.truncate(0)

    #position file
    pos_file = open("position.txt","a")
    pos_file.truncate(0)

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

    # raw angles in radians
    lg_lh = LogConfig(name='lighthouse', period_in_ms=100)
    lg_lh.add_variable('lighthouse.angle0x_0lh2', 'float')
    lg_lh.add_variable('lighthouse.angle0y_0lh2', 'float')
    lg_lh.add_variable('lighthouse.angle1x_0lh2', 'float')
    lg_lh.add_variable('lighthouse.angle1y_0lh2', 'float')
    lg_lh.add_variable('lighthouse.bsGeoVal','uint16_t')

    # lg_lh = LogConfig(name='lighthouse', period_in_ms=100)
    # lg_lh.add_variable('lighthouse.angle0x', 'float')
    # lg_lh.add_variable('lighthouse.angle0y', 'float')
    # lg_lh.add_variable('lighthouse.angle1x', 'float')
    # lg_lh.add_variable('lighthouse.angle1y', 'float')
    # lg_lh.add_variable('lighthouse.bsGeoVal','uint16_t')

    ReadMem(uri)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        #cross-beam method to get lighthouse.x,y,z
        cf = scf.cf
        param_name = "lighthouse.method"
        param_value = 0
        cf.param.set_value(param_name, param_value)
        simple_log_async(scf, lg_stab, lg_lh, lg_pos)

    for i in my_pos_coords:
        update_line(hl, list(i))
        plt.show(block=False)

    for i in their_pos_coords:
        update_line(hb, list(i))
        plt.show(block=False)

    plt.show(block=True)

    raw_angles_file.close()
    ypr_file.close()
    pos_file.close()

    


