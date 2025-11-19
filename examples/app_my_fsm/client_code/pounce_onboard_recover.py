"""
    Repeat tests back-to-back to:

        1.  Observe how accurately/precisely I can place hopcopter on the ground

        2.  Compensate for landing position overshoot by correcting X_APEX, the point
            at which the flight controller switches to attitude-only control.

    NOTE:   
    
            -   This version of the flight code is supposed to be more readable and more general

            -   I have integrated code to stream the position of the soccerball from motion capture

            -   Trying to calibrate for different drop heights, lateral velocities

            -   Trying to land at steeper angles

"""

import os
import datetime
import time
import json
import logging
import signal
import numpy as np

from threading import Event

from scipy.spatial.transform import Rotation

from cflib import crtp
from CrazyflieClient import CrazyflieClient

from MotionCaptureForwardPoses import QualisysStreamProcessor

from hash_helper import get_git_hash

# Set up logging
logging.basicConfig(level=logging.INFO)

main_logger = logging.getLogger(__name__)
main_logger.setLevel(logging.INFO)

EMERGENCY_STOP_EVENT = Event()

# User-defined parameters
QTM_SERVER_ADDRESS = '128.174.245.64'          # Your QTM server IP
RIGID_BODY_NAME = "cf1"                   # Your rigid body name
DESIRED_FREQUENCY = 100                         # Desired frequency in Hz
URI = 'radio://0/55/2M/E7E7E7E7E7'              # Your Crazyflie URI
DRONE_LOG_VARIABLES = [
    'stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z',
    'stateEstimate.yaw', 'stateEstimate.pitch', 'stateEstimate.roll',
    'stateEstimate.vx', 'stateEstimate.vy', 'stateEstimate.vz',
    'gyro.x', 'gyro.y', 'gyro.z',
    'acc.x', 'acc.y', 'acc.z',
    'ctrltarget.x', 'ctrltarget.y', 'ctrltarget.z',
    'ctrltarget.roll', 'ctrltarget.pitch', 'ctrltarget.yaw', 'ctrltarget.yawAbs', 'stabilizer.thrust',
    'controller.cmd_thrust', 'controller.cmd_roll', 'controller.cmd_pitch', 'controller.cmd_yaw',
    'controller.yaw',
    'motor.m1', 'motor.m2', 'motor.m3', 'motor.m4',
    'pid_freefall.pitch_outP', 'pid_freefall.pitch_outD',
    'pid_freefall.roll_outP', 'pid_freefall.roll_outD',
    'pid_freefall.yaw_outP', 'pid_freefall.yaw_outD',
    'my_fsm.curr_state'
]

MARKER_DECK_IDS = [1, 2, 3, 4]

# The max lateral velocity. This is also assumed to be the
# lateral velocity at the apex of the ballistic phase.
V_MAX       = 3.3           # m/s
Z_APEX      = 0.78           # m

#   Overshoot formula parameters
OVERSHOOT_SLOPE = 0.45
OVERSHOOT_INTERCEPT =  0.0055

#   Recovery PD attitude control gains
RECOVER_ROLL_KP     =   180 
RECOVER_ROLL_KD     =   85
RECOVER_PITCH_KP    =   180
RECOVER_PITCH_KD    =   85

class QualisysForwardPose(QualisysStreamProcessor):
    """

    This class is used to forward the pose of the drone to the Crazyflie client.
    It is a subclass of the QualisysStreamProcessor class.
    '_process_body_data' is overridden to send the pose of the drone to the Crazyflie client.

    """

    def __init__(self, ip_address: str, rigid_bodies: list[str], frequency: int, cf_client: CrazyflieClient):
        super().__init__(ip_address, rigid_bodies, frequency)
        self.cf_client = cf_client

    def _process_body_data(self, body_name: str, pose_data: dict):
        body_data = pose_data.get(body_name)
        if body_data:
            if body_data['t'] != -1:  # Check if the body was detected in this frame
                position = body_data['p']
                quaternion = Rotation.from_matrix(body_data['R']).as_quat()
                self.cf_client.cf.extpos.send_extpose(*position, *quaternion)
            else:
                # print(f"\r{body_name} not found in current frame", end="")
                pass

def emergency_stop(sig, frame):
    print("Emergency stop")
    EMERGENCY_STOP_EVENT.set()

def compute_freefall_pos(p_des: list[float, float], v_max: float, z_apex: float):
    """
    Compute the maximum x-position at the apex of the ballistic phase.
    """

    if z_apex < 0.0:
        raise

    #   Convert lists to numpy arrays for quick math
    p_des = np.array(p_des)

    #   Compute the unit vector from current to desired landing position
    n_vel = p_des / np.linalg.norm(p_des)

    return p_des - v_max * np.sqrt(2*z_apex/9.81) * n_vel 

def compute_overshoot(p_des: list[float, float], v_max: float):

    #   Convert lists to numpy arrays for quick math
    p_des = np.array(p_des)

    #   Compute the unit vector from current to desired landing position
    n_vel = p_des / np.linalg.norm(p_des)

    #   Use this formula if you want to perform calibration and determine
    #   the regression coefficients
    # return np.array([0, 0])

    # #   Used this formula for the exercise ball with some success
    # return 0.50 * v_max * n_vel + 0.0055


    #   Parameterize the formula; last known working values
    return OVERSHOOT_SLOPE * v_max * n_vel + OVERSHOOT_INTERCEPT


def switch_to_freefall(p_curr: list[float, float], p_apex: list[float, float], p_far):

    p_curr = np.array(p_curr)
    p_apex = np.array(p_apex)
    p_far = np.array(p_far)

    n_vel = p_far / np.linalg.norm(p_far)

    if np.dot((p_curr - p_apex), n_vel) > 0:
        return True
    
    else:
        return False

def compute_desired_pitch_roll(z: float, vx: float, vy: float, vz: float) -> tuple[float, float]:
    """
    Compute the desired pitch and roll for the ballistic phase.

    Outputs are in degrees.
    """
    T_fall = np.sqrt(2*z/9.81)

    v_land = np.array([vx, vy, vz-9.81*T_fall])

    body_z_axis_des = -v_land/np.linalg.norm(v_land)

    c13, c23, _ = body_z_axis_des

    pitch_des = np.arcsin(c13)                      # Radian
    roll_des = np.arcsin(-c23/np.cos(pitch_des))    # Radian

    return np.rad2deg(roll_des), np.rad2deg(pitch_des)


def initialize_systems(config):
    """Initialize Crazyflie and motion capture systems"""
    crtp.init_drivers()
    
    cf = CrazyflieClient(
        uri=config.URI, 
        log_variables=config.DRONE_LOG_VARIABLES, 
        marker_deck_ids=config.MARKER_DECK_IDS
    )
    
    # Wait for connection
    while not cf.is_fully_connected:
        time.sleep(0.1)

    cf.cf.param.set_value("my_fsm.rec_pKp", RECOVER_PITCH_KP)
    time.sleep(0.1)
    cf.cf.param.set_value("my_fsm.rec_pKd", RECOVER_PITCH_KD)
    time.sleep(0.1)

    cf.cf.param.set_value("my_fsm.rec_rKp", RECOVER_ROLL_KP)
    time.sleep(0.1)
    cf.cf.param.set_value("my_fsm.rec_rKd", RECOVER_ROLL_KD)
    time.sleep(0.1)
    
    qtm_thread = QualisysForwardPose(
        ip_address=config.QTM_SERVER_ADDRESS,
        rigid_bodies=[config.RIGID_BODY_NAME],
        frequency=config.DESIRED_FREQUENCY,
        cf_client=cf
    )
    
    return cf, qtm_thread

if __name__ == '__main__':

    config = type('Config', (), {
        'URI': URI,
        'DRONE_LOG_VARIABLES': DRONE_LOG_VARIABLES,
        'MARKER_DECK_IDS': MARKER_DECK_IDS,
        'QTM_SERVER_ADDRESS': QTM_SERVER_ADDRESS,
        'RIGID_BODY_NAME': RIGID_BODY_NAME,
        'DESIRED_FREQUENCY': DESIRED_FREQUENCY,
        'V_MAX': V_MAX,
        'Z_APEX': Z_APEX
    })()

    cf, qtm_thread = initialize_systems(config)

    # Register the emergency stop handler
    signal.signal(signal.SIGINT, emergency_stop)

    # This sleep is crucial. DO NOT REMOVE IT.
    cf.stop(5.0)

    # Get the initial position of the drone from the QTM server.
    init_pos = np.array([np.nan, np.nan, np.nan])
    while not np.isfinite(init_pos[0]):
        init_pos[0] = qtm_thread.client.data[RIGID_BODY_NAME]['x'][-1]
        init_pos[1] = qtm_thread.client.data[RIGID_BODY_NAME]['y'][-1]
        init_pos[2] = qtm_thread.client.data[RIGID_BODY_NAME]['z'][-1]
        time.sleep(0.5)

    print(f"Initial drone position: {init_pos}")

    init_soccer_pos = np.array([np.nan, np.nan, np.nan])
    while not np.isfinite(init_soccer_pos[0]):
        init_soccer_pos[0] = qtm_thread.client.data['soccerball']['x'][-1]
        init_soccer_pos[1] = qtm_thread.client.data['soccerball']['y'][-1]
        init_soccer_pos[2] = qtm_thread.client.data['soccerball']['z'][-1]        
        time.sleep(0.5)

    print(f"Initial soccerball position: {init_soccer_pos}")

    # Bounce state machine
    state = 'init'
    start = time.time()

    # Compute trajectory quantities
    P_LAND_DES =          init_soccer_pos[0:2] - init_pos[0:2]
    P_SWITCH =            compute_freefall_pos(P_LAND_DES, V_MAX, Z_APEX-init_soccer_pos[2]) + init_pos[0:2]
    P_SWITCH_CORRECTED =  P_SWITCH - compute_overshoot(P_LAND_DES, V_MAX)

    while True:
        # break
        # Break if emergency stop is triggered
        if EMERGENCY_STOP_EVENT.is_set():
            break

        # Collect conditions
        wx, wy, wz = cf.data['gyro.x']['data'][-1], cf.data['gyro.y']['data'][-1], cf.data['gyro.z']['data'][-1]
        x, y, z = cf.data['stateEstimate.x']['data'][-1], cf.data['stateEstimate.y']['data'][-1], cf.data['stateEstimate.z']['data'][-1]
        vx, vy, vz = cf.data['stateEstimate.vx']['data'][-1], cf.data['stateEstimate.vy']['data'][-1], cf.data['stateEstimate.vz']['data'][-1]
        az = cf.data['acc.z']['data'][-1]

        if state == 'init':
            cf.cf.high_level_commander.takeoff(Z_APEX, 3.0)
            time.sleep(5)
            state = 'homed1'

        elif state == 'homed1':


            dp = init_pos[0:2] - init_soccer_pos[0:2]
            n_des = dp / np.linalg.norm(dp)
            yaw_des = np.asin(n_des[1])

            #   Adjust quadrotor yaw to make it face the target
            cf.cf.high_level_commander.go_to(x=0, y=0, z=0, yaw=yaw_des, duration_s=1.5, relative=True)


            time.sleep(2)
            state = 'homed'

        elif state == 'homed':
            
            #   In order to lunge/leap towards the target, command a really far away (5x away)
            #   position setpoint for the drone. The time it takes to go there must correspond 
            #   to the desired leaping velocity
            
            P_far = [5*P_LAND_DES[0], 5*P_LAND_DES[1]]
            P_far_dist = np.linalg.norm(P_far)
            time_to_far_point = P_far_dist/V_MAX
            cf.cf.high_level_commander.go_to(P_far[0], P_far[1], 0.0, 0.0, time_to_far_point, linear=True, relative=True)
            state = 'leaping'
        
        elif state == 'leaping':

            pos_curr = [x, y]
            P_far = [5*P_LAND_DES[0], 5*P_LAND_DES[1]]

            if switch_to_freefall(pos_curr, P_SWITCH_CORRECTED, P_far):

                state = 'ballistic'
                cf.cf.commander.send_setpoint(0, 0, 0, 0)
                # roll_final, pitch_final = compute_desired_pitch_roll(z-init_soccer_pos[2], -4.0*V_MAX, 0.0, vz)


        elif state == 'ballistic':

            # roll_des = roll_final  
            # pitch_des = pitch_final
            roll_des    =   0.0
            pitch_des   =   50.0
            cf.cf.commander.send_setpoint(roll_des, pitch_des, yaw_des, 1001)
            # cf.cf.commander.send_setpoint(roll_des, pitch_des, 0.0, 1001)
            # print(f"roll_des: {roll_des}, pitch_des: {pitch_des}")

            # If CF crashed/bounced, recover.
            if az > 2.5:

                # When using the custom FSM to determine impact/bounce,
                # the quadrotor is autonomous in leveling out. Simply exit 
                # the loop and call time.sleep()
                
                break
                # t_start_level = time.time()
                # state = 'levelout'

        
        elif state == 'pause':

            if (time.time() - t_start_level) > 0.01:
                print('foo')
                state = 'levelout'
                t_start_level = time.time()

        elif state == 'levelout':

            roll_level, pitch_level, yaw_level = 0, 0, 0
            cf.cf.commander.send_setpoint(roll_level, pitch_level, yaw_level, 48000)

            if (time.time() - t_start_level) > 3.0:
                break

        main_logger.info(f"State: {state}")
        time.sleep(0.01)
 
    # Pause AT LEAST 5s to save some crash data.
    # May be helpful in debugging
    # cf.stop(5.0)

    time.sleep(5.0)


    cf.disconnect()
    qtm_thread.stop()

    #   Build metadata
    CODE_VERSION = get_git_hash(repo_path="../../../")
    script_name = os.path.basename(__file__)
    script_base_name = os.path.splitext(script_name)[0]
    TIMESTAMP = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")

    #   Build parameter data for this specific run
    parameters_dict = {
        "V_MAX": V_MAX,
        "P_LAND_DES": P_LAND_DES.tolist(),
        "Z_APEX": Z_APEX,
        "OVERSHOOT_SLOPE": OVERSHOOT_SLOPE,
        "OVERSHOOT_INTERCEPT": OVERSHOOT_INTERCEPT,
        "RECOVER_ROLL_KP": RECOVER_ROLL_KP,
        "RECOVER_ROLL_KD": RECOVER_ROLL_KD,
        "RECOVER_PITCH_KP": RECOVER_PITCH_KP,
        "RECOVER_PITCH_KD": RECOVER_PITCH_KD
    }

    #   Version, timestamp, parameters
    metadata_dict = {
        "code_version": CODE_VERSION,
        "script_name": script_name,
        "timestamp": TIMESTAMP,
        "parameters": parameters_dict}

    full_logs = {
        "data": cf.data,
        "metadata": metadata_dict,
        "mocap": qtm_thread.client.data
    }

    log_folder = "flight_logs/"
    if not os.path.exists(log_folder):
        os.makedirs(log_folder)

    log_filename = os.path.join(
        log_folder, 
        f"log_{CODE_VERSION}_{script_base_name}_{TIMESTAMP}.json"
    )

    with open(log_filename, 'w') as f:
        json.dump(full_logs, f, indent=4)

    print(f"Log saved to: {log_filename}")