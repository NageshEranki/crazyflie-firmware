"""
-   Contains the CrazyflieClient class definition.
-   Import this class into a 'flight' script and create instances of the class.
-   Handles the following:
    * Radio connection
    * Logging drone variables
    * Forwarding localisation data to the state-estimator
    * Sending control setpoints

-   This class has a member variable 'data' of dict type. It is up to the user to save this to a
    suitable file format.

-   Args:
    * uri (str)                 : The Uniform Resource Identifier, denoting the radio channel
    * use_controller (str)      : A choice from 'PID', 'Mellinger', 'AE483'
    * log_variables (list[str]) : List of drone variables to log to member variable 'data'
    * marker_deck_ids (list[int])
    * cache_path (str)          : (ignore)
    * debug_console (bool)      : Whether to output console channel output from drone to logger.

-   Methods:
    * move(x, y, z, yaw, dt)    : Go-to (x, y, z, yaw, dt) for 'dt' seconds.
    * stop(dt):                 : Stop motors, cancelling the current setpoint and wait for 'dt' seconds.
    * disconnect()              : Disconnect the radio link. This must be called at the end of your flight.
"""

# Imports for crazyflie (the drone)
import logging
import time
import struct
import numpy as np
import json
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

cf_logger = logging.getLogger('CrazyflieClient')

class CrazyflieClient:
    def __init__(self, uri, use_controller='PID', log_variables=[], marker_deck_ids=None, cache_path=None, debug_console=False):
        """
        Initialize a CrazyflieClient.

        Args:
            uri (str): URI for connecting to the Crazyflie
            use_controller (str): Controller type ('PID', 'mellinger', or 'AE483')
            log_variables (list): List of variables to log
            marker_deck_ids (list): List of marker IDs for active marker deck
            cache_path (str, optional): Path to the cache directory. If None, creates a '.cache' 
                                      directory in the current working directory.
        """
        assert use_controller in ['PID', 'mellinger', 'AE483']
        self.use_controller = use_controller
        self.marker_deck_ids = marker_deck_ids
        self.variables = log_variables
        self.debug_console = debug_console

        self.cf = Crazyflie(rw_cache='./cache')
        self.cf.connected.add_callback(self._connected)
        if self.debug_console:
            self.cf.console.receivedChar.add_callback(self._log_from_console)
        self.cf.fully_connected.add_callback(self._fully_connected)
        self.cf.connection_failed.add_callback(self._connection_failed)
        self.cf.connection_lost.add_callback(self._connection_lost)
        self.cf.disconnected.add_callback(self._disconnected)
        # self.cf.appchannel.packet_received.add_callback(self.app_packet_received)
        # self.app_thread = Thread(target=self.app_send_data)
        print(f'CrazyflieClient: Connecting to {uri}')
        self.cf.open_link(uri)
        self.is_fully_connected = False
        self.data = {}

    def _connected(self, uri):
        print(f'CrazyflieClient: Connected to {uri}')
    
    def _fully_connected(self, uri):
        if self.marker_deck_ids is not None:
            print(f'CrazyflieClient: Using active marker deck with IDs {self.marker_deck_ids}')

            # Set the marker mode (3: qualisys)
            self.cf.param.set_value('activeMarker.mode', 3)

            # Set the marker IDs
            self.cf.param.set_value('activeMarker.front', self.marker_deck_ids[0])
            self.cf.param.set_value('activeMarker.right', self.marker_deck_ids[1])
            self.cf.param.set_value('activeMarker.back',  self.marker_deck_ids[2])
            self.cf.param.set_value('activeMarker.left',  self.marker_deck_ids[3])

        # Select the controller
        if self.use_controller == 'PID':
            self.cf.param.set_value('stabilizer.controller', 1)
        elif self.use_controller == 'mellinger':
            self.cf.param.set_value('stabilizer.controller', 2)
        elif self.use_controller == 'AE483':
            self.cf.param.set_value('stabilizer.controller', 6)

        # Select the estimator
        self.cf.param.set_value('stabilizer.estimator', 2)

        # Reset the default observer
        self.cf.param.set_value('kalman.resetEstimation', 1)
        time.sleep(0.1)
        self.cf.param.set_value('kalman.resetEstimation', 0)

        self.cf.param.set_value('flightmode.stabModeYaw', 1)
        
        # Start logging
        self.logconfs = []
        self.logconfs.append(LogConfig(name=f'LogConf0', period_in_ms=10))
        num_variables = 0
        for v in self.variables:
            num_variables += 1
            if num_variables > 5: # <-- could increase if you paid attention to types / sizes (max 30 bytes per packet)
                num_variables = 0
                self.logconfs.append(LogConfig(name=f'LogConf{len(self.logconfs)}', period_in_ms=10))
            self.data[v] = {'time': [], 'data': []}
            self.logconfs[-1].add_variable(v)
        for logconf in self.logconfs:
            try:
                self.cf.log.add_config(logconf)
                logconf.data_received_cb.add_callback(self._log_data)
                logconf.error_cb.add_callback(self._log_error)
                logconf.start()
            except KeyError as e:
                print(f'CrazyflieClient: Could not start {logconf.name} because {e}')
                for v in logconf.variables:
                    print(f' - {v.name}')
            except AttributeError:
                print(f'CrazyflieClient: Could not start {logconf.name} because of bad configuration')
                for v in logconf.variables:
                    print(f' - {v.name}')
        
        print(f'CrazyflieClient: Fully connected to {uri}')
        self.is_fully_connected = True
        # self.app_thread.start()

    def _connection_failed(self, uri, msg):
        print(f'CrazyflieClient: Connection to {uri} failed: {msg}')

    def _connection_lost(self, uri, msg):
        print(f'CrazyflieClient: Connection to {uri} lost: {msg}')

    def _disconnected(self, uri):
        for logconf in self.logconfs:
            logconf.stop()
        print(f'CrazyflieClient: Disconnected from {uri}')
        self.is_fully_connected = False
    
    def _log_data(self, timestamp, data, logconf):
        for v in logconf.variables:
            self.data[v.name]['time'].append(timestamp / 1e3)
            self.data[v.name]['data'].append(data[v.name])

    def _log_error(self, logconf, msg):
        print(f'CrazyflieClient: Error when logging {logconf}: {msg}')

    def _log_from_console(self, text):
        # print(f"CONSOLE: {text}")
        cf_logger.info(text)

    def app_packet_received(self, data):
        (sum, ) = struct.unpack("<f", data)
        print(f"Received sum: {sum}")

    def app_send_data(self):
        i = 0
        while True:
            if not self.is_fully_connected:
                break
            (x, y, z) = (i, i+1, i+2)
            data = struct.pack("<fff", x, y, z)
            self.cf.appchannel.send_packet(data)
            print(f"Sent x: {x}, y: {y}, z: {z}")

            i += 1

            time.sleep(1)        

    def move(self, x, y, z, yaw, dt):
        print(f'CrazyflieClient: Move to {x}, {y}, {z} with yaw {yaw} degrees for {dt} seconds')
        start_time = time.time()
        while time.time() - start_time < dt:
            self.cf.commander.send_position_setpoint(x, y, z, yaw)
            time.sleep(0.1)
    
    def move_by_vel(self, p1: list, p2: list, dt: float):
        p1 = np.array(p1)
        p2 = np.array(p2)
        t_start = time.time()
        s = 0
        while s <= 1:
            p = p1 + s * (p2 - p1)
            self.cf.commander.send_position_setpoint(p[0], p[1], p[2], 0)
            time.sleep(0.05)
            s = (time.time() - t_start)/dt
        self.cf.commander.send_notify_setpoint_stop()
    
    def send_vel_setpoint(self, vx, vy, vz, yawrate, dt):
        start = time.time()
        while time.time() - start < dt:
            self.cf.commander.send_velocity_world_setpoint(vx, vy, vz, yawrate)
            time.sleep(0.05)
        self.cf.commander.send_notify_setpoint_stop()
    
    def stop(self, dt):
        print(f'CrazyflieClient: Stop for {dt} seconds')
        self.cf.commander.send_stop_setpoint()
        self.cf.commander.send_notify_setpoint_stop()
        start_time = time.time()
        while time.time() - start_time < dt:
            time.sleep(0.1)

    def disconnect(self):
        self.cf.close_link()
        # with open('data.json', 'w') as fh:
        #     json.dump(self.data, fh)
        # self.app_thread.join()