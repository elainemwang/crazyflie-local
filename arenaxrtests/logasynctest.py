"""
Simple example that connects to the first Crazyflie found, logs the Stabilizer
and prints it to the console. After 10s the application disconnects and exits.
From Bitcraze example code: https://github.com/bitcraze/crazyflie-lib-python/blob/master/examples/logging/basiclog.py
"""
import logging
import time
from threading import Timer, Thread

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
import math
from pynput import keyboard


from arena import *
scene = Scene(host="mqtt.arenaxr.org", scene="crazyflie", namespace="emwang2")
drone = GLTF(object_id="drone", position=Position(0,0,0), scale=Scale(1,1,1), url="store/models/Drone.glb")
scene.add_object(drone)

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

class StateLogger:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=100)
        self._lg_stab.add_variable('stateEstimate.x', 'float')
        self._lg_stab.add_variable('stateEstimate.y', 'float')
        self._lg_stab.add_variable('stateEstimate.z', 'float')
        self._lg_stab.add_variable('stabilizer.roll', 'float')
        self._lg_stab.add_variable('stabilizer.pitch', 'float')
        self._lg_stab.add_variable('stabilizer.yaw', 'float')
        # The fetch-as argument can be set to FP16 to save space in the log packet
        self._lg_stab.add_variable('pm.vbat', 'FP16')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

        # Start a timer to disconnect in 10s
        # New: emwang2 - use q to disconnect from crazyflie
        # t = Timer(30, self._cf.close_link)
        # t.start()

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        print(f'[{timestamp}][{logconf.name}]: ', end='')
        for name, value in data.items():
            print(f'{name}: {value:3.3f} ', end='')

        x = data["stateEstimate.x"]
        y = data["stateEstimate.y"]
        z = data["stateEstimate.z"]

        if z > 0.5:
            thrust = 30000
            while thrust:
                self._cf.commander.send_setpoint(0, 0, 0, thrust)
                thrust -= 500
                time.sleep(0.1)
            self._cf.close_link()

        roll = data["stabilizer.roll"]
        pitch = data["stabilizer.pitch"]
        yaw = data["stabilizer.yaw"]
        scale = 1

        drone.update_attributes(position=Position(y*scale,z*scale,x*scale))
        drone.update_attributes(rotation=Rotation(math.radians(-pitch),math.radians(yaw),math.radians(roll)))
        scene.update_object(drone)

        print()


    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False

    def _ramp_motors(self):
        thrust_mult = 1
        thrust_step = 500
        thrust = 20000
        pitch = 0
        roll = 0
        yawrate = 0

        # Unlock startup thrust protection
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        while thrust >= 20000:
            self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
            time.sleep(0.1)
            if thrust >= 50000:
                thrust_mult = -1
            thrust += thrust_step * thrust_mult
        self._cf.commander.send_setpoint(0, 0, 0, 0)

def on_press(key, le):
    try:
        if key.char == 'e':
            Thread(target=le._ramp_motors).start()
        if key.char == 'q':
            # Make sure that the last packet leaves before the link is closed
            # since the message queue is not flushed before closing
            time.sleep(0.1)
            le._cf.close_link()
            return False
    except AttributeError:
        print('special key {0} pressed'.format(
            key))

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    sl = StateLogger(uri)

    listener = keyboard.Listener(
        on_press=lambda event: on_press(event, sl))
    listener.start()

    scene.run_tasks()
    
    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.
    # while sl.is_connected:
    #     time.sleep(1)
    