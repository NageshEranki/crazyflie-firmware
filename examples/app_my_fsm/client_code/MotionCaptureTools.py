"""
This is a template for a QualisysClient.
It is used to connect to a Qualisys motion capture system and stream data to a queue.
"""
import itertools
from threading import Thread
from queue import SimpleQueue
import asyncio
import numpy as np
import xml.etree.cElementTree as ET
import qtm_rt as qtm
import json
import logging

qtm_logger = logging.getLogger('qtm_rt')

def circumcenter(point_array):
    """
        Compute the center of the circumsphere from 4 given points on a sphere.

        Inputs:

            -- point_array : numpy.ndarry, shape = (N, 3)

        Output:

            -- c : numpy.ndarray, shape = (3,)
    """
    A = point_array[1:,:]-point_array[0,:]

    b = 0.5 * (np.linalg.norm(point_array[1:,:], axis=1)**2 - np.linalg.norm(point_array[0,:])**2)

    if np.linalg.cond(A) > 100:
        return

    # print(np.linalg.det(A))
    
    return np.linalg.solve(A, b)

class QualisysClient(Thread):
    """
        Class definition for a listener Thread; All arguments are required.

            - ip_address:       IP address of the PC running QTM

            - rigid_bodies:     list of rigid body names you want to track

            - pose_queue:       A SimpleQueue instance. Motion capture data is placed on this
                                
                                WARNING: Data is not put on the queue if it not empty. It is the user's responsibility to quickly dequeue data.

            - freq_des:         Desired capture rate; This thread will modify QTM settings to change capture rate.


        NOTES:
                            - Motion capture data for each rigid body is streamed directly from QTM without any relative transformations.
                            - Position data is in meters (converted from QTM's millimeters).
                            - Orientation data is provided as a rotation matrix.
                            - Data is assembled into a nested dictionary with rigid body names as keys.
                            - Any client reading data off the queue should check if t = -1, which indicates the rigid body
                              was not observed in the current packet.
    """
    def __init__(self, ip_address: str, rigid_bodies: list[str], pose_queue: SimpleQueue, freq_des: int):
        Thread.__init__(self)
        self.ip_address = ip_address
        self.rigid_bodies = rigid_bodies
        self.connection = None
        self.qtm_6DoF_labels = []
        self._stay_open = True
        self.data = {
            body_name:{
                'time': [],
                'x': [],
                'y': [],
                'z': [],
                'rotation_matrix': []} for body_name in rigid_bodies
        }
        self.data['soccerball'] = {
            'x':[],
            'y':[],
            'z':[],
            'time':[],
            'R':[]
        }
        
        self.pose_queue = pose_queue
        self.freq_des = freq_des
        self.start()

    def close(self):
        self.pose_queue.put('END')
        self._stay_open = False
        self.join()

    def run(self):
        asyncio.run(self._life_cycle())

    async def _life_cycle(self):
        await self._connect()
        while (self._stay_open):
            await asyncio.sleep(1)
        # Save data before closing thread
        # with open('motion_capture_data.json', 'w') as outfile:
        #     json.dump(self.data, outfile, sort_keys=False)
        
        await self._close()

    async def change_qtm_freq(self):
        params = await self.connection.get_parameters(parameters=['general'])
        xml = ET.fromstring(params)

        for freq in xml.findall('*/Frequency'):
            freq.text = str(self.freq_des)

        xml.tag = "QTM_Settings"
        msg = await self.connection.send_xml(ET.tostring(xml).decode("utf-8"))

    async def _connect(self):
        print('QualisysClient: Connect to motion capture system')
        self.connection = await qtm.connect(self.ip_address, version='1.24')
        await self.connection.take_control("")

        await self.change_qtm_freq()

        params = await self.connection.get_parameters(parameters=['6d'])
        xml = ET.fromstring(params)
        self.qtm_6DoF_labels = [label.text.strip() for _, label in enumerate(xml.findall('*/Body/Name'))] # type: ignore

        await self.connection.stream_frames(
            components=['6d', '3dnolabels'],
            on_packet=self._on_packet_all,
        )

    def _on_packet_all(self, packet):
        self._on_packet(packet)
        self._on_marker_packet(packet)

    def _on_packet(self, packet):
        _, bodies = packet.get_6d()
        
        # Get time in seconds, with respect to the qualisys clock
        t = packet.timestamp / 1e6
        
        if bodies is None:
            print(f'QualisysClient: No rigid bodies found')
            return

        extpos_packet = {body_name:
            {
                'p': [np.nan, np.nan, np.nan],
                'R': np.eye(3),
                't': -1
            }
            for body_name in self.rigid_bodies
        }

        for body_name in self.rigid_bodies:
            
            if body_name not in self.qtm_6DoF_labels:
                ## DO NOT return , you can still do useful stuff even if a few bodies are not found
                ## Do however, warn the user that they are missing!
                print(f'QualisysClient: Marker deck {body_name} not found')
                continue
        
            index = self.qtm_6DoF_labels.index(body_name)
            position, orientation = bodies[index]

            try:
                # Get position in meters (converting from millimeters)
                x, y, z = np.array(position) / 1e3

                # Get orientation as rotation matrix
                R = np.reshape(orientation.matrix, (3, -1), order='F')
            except np.linalg.LinAlgError as err:
                print(err)

            # Store time, position, and orientation
            self.data[body_name]['time'].append(t)
            self.data[body_name]['x'].append(x)
            self.data[body_name]['y'].append(y)
            self.data[body_name]['z'].append(z)
            self.data[body_name]['rotation_matrix'].append(R.tolist())

            # Check if the measurements are valid
            if np.isfinite(x):
                extpos_packet[body_name]['p'] = np.array([x, y, z])
                extpos_packet[body_name]['R'] = R
                extpos_packet[body_name]['t'] = t

        if self.pose_queue.empty():
            # Put pose in queue to send to the client
            self.pose_queue.put(extpos_packet)

    def _on_marker_packet(self, packet):
        _, markers = packet.get_3d_markers_no_label()

        if markers is None:
            print(f'QualisysClient: No markers found')
            return
        
        if len(markers) < 4:
            print("Too few markers are visible. I cannot compute the ball's center")
            return
         

        # Get time in seconds, with respect to the qualisys clock
        t = packet.timestamp / 1e6

        center = None
        combinations = itertools.combinations(markers, 4)
        while center is None:
            try:
                marker_set = next(combinations)

                point_array = np.array([
                    marker_set[0].x, marker_set[0].y, marker_set[0].z,
                    marker_set[1].x, marker_set[1].y, marker_set[1].z,
                    marker_set[2].x, marker_set[2].y, marker_set[2].z,
                    marker_set[3].x, marker_set[3].y, marker_set[3].z,
                ]).reshape(-1, 3) / 1e3

                center = circumcenter(point_array)
                
            except StopIteration as e:
                print(e)
                break

        if center is None:
            print("Bad condition number.")
            return
        
        else:
            self.data['soccerball']['x'].append(center[0])
            self.data['soccerball']['y'].append(center[1])
            self.data['soccerball']['z'].append(center[2])
            self.data['soccerball']['R'].append(np.mean(np.linalg.norm(point_array - center, axis=1)))
            self.data['soccerball']['time'].append(t)

    async def _close(self):
        await self.connection.stream_frames_stop() # type: ignore
        self.connection.disconnect() # type: ignore


class FakeQualisysClient(Thread):
    def __init__(self, filename, rigid_body_name):
        Thread.__init__(self)
        self.filename = filename
        self.rigid_body_name = rigid_body_name