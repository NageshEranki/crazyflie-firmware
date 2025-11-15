/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * my_fsm.c : A simple finite state machine (FSM) that keeps track of the 
 *            accelerometer data to detect impacts and recover the quadrotor.
 */


#include <string.h>
// #include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "num.h"
#include "math3d.h"
#include "app.h"
#include "log.h"
#include "commander.h"
#include "crtp_commander_high_level.h"
#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "FSM"

#include "debug.h"
#include "my_fsm.h"




StateType state = NORMAL;

// Quadrotor states
static float acc_z;
static float px;
static float py;
static float pz;

static float vx;
static float vy;
static float vz;

static float qx;
static float qy;
static float qz;
static float qw;

static float wx;
static float wy;
static float wz;

static float yaw;


//  Hold states
static float px_hold;
static float py_hold;
static float pz_hold;
static float yaw_hold;

// Logging IDs (initialize/declaration only)
static logVarId_t idAccZ; 

static logVarId_t idPx;
static logVarId_t idPy;   
static logVarId_t idPz;

static logVarId_t idvx;
static logVarId_t idvy;   
static logVarId_t idvz;

static logVarId_t idqx;
static logVarId_t idqy;   
static logVarId_t idqz;
static logVarId_t idqw;

static logVarId_t idYaw;  

static logVarId_t idwx;
static logVarId_t idwy;   
static logVarId_t idwz;




void appSetup()
{
  //  Extract the log IDs for required quadrotor states
  idAccZ  =   logGetVarId("acc", "z");

  idPx    =   logGetVarId("stateEstimate", "x");
  idPy    =   logGetVarId("stateEstimate", "y");
  idPz    =   logGetVarId("stateEstimate", "z");

  idvx    =   logGetVarId("stateEstimate", "vx");
  idvy    =   logGetVarId("stateEstimate", "vy");
  idvz    =   logGetVarId("stateEstimate", "vz");

  idYaw   =   logGetVarId("stateEstimate", "yaw");

  idqx    =   logGetVarId("stateEstimate", "qx");
  idqy    =   logGetVarId("stateEstimate", "qy");
  idqz    =   logGetVarId("stateEstimate", "qz");
  idqw    =   logGetVarId("stateEstimate", "qw");

  idwx    =   logGetVarId("gyro", "x");
  idwy    =   logGetVarId("gyro", "y");
  idwz    =   logGetVarId("gyro", "z"); 
}




void pullStates()
{
  //  Read the current values for the all the logging variables
  px      = logGetFloat(idPx);
  py      = logGetFloat(idPy);
  pz      = logGetFloat(idPz);

  vx      = logGetFloat(idvx);
  vy      = logGetFloat(idvy);
  vz      = logGetFloat(idvz);

  wx      = radians(logGetFloat(idwx));
  wy      = radians(logGetFloat(idwy));
  wz      = radians(logGetFloat(idwz));
  
  qx      = logGetFloat(idqx);
  qy      = logGetFloat(idqy);
  qz      = logGetFloat(idqz);
  qw      = logGetFloat(idqw);

  yaw     = logGetFloat(idYaw);

  acc_z   = logGetFloat(idAccZ);
}



void setRecoverSetpoint(setpoint_t* setpoint, const float roll, const float pitch, const float yaw, const uint16_t thrust)
{

  setpoint->mode.x = modeDisable;
  setpoint->mode.y = modeDisable;

  //  Added this after data8.json onwards.
  //  The stock PID controller will process
  //  manual thrust setpoints only if
  //  mode is set to modeDisable (see code)
  setpoint->mode.z = modeDisable;

  setpoint->mode.roll = modeAbs;
  setpoint->attitudeRate.roll = 0;
  setpoint->attitude.roll = roll;

  setpoint->mode.pitch = modeAbs;
  setpoint->attitudeRate.pitch = 0;
  setpoint->attitude.pitch = pitch;
  
  setpoint->mode.yaw = modeAbs;
  setpoint->attitudeRate.yaw = 0;
  setpoint->attitude.yaw = yaw;
  
  setpoint->thrust = fminf(thrust, MAX_THRUST);
  
}

void setHoldSetpoint(setpoint_t* setpoint, const float x, const float y, const float z, const float yaw)
{

  setpoint->mode.x = modeAbs;
  setpoint->position.x = x;

  setpoint->mode.y = modeAbs;
  setpoint->position.y = y;

  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;  
  
  setpoint->mode.yaw = modeAbs;
  setpoint->attitudeRate.yaw = 0;
  setpoint->attitude.yaw = yaw;
    
}


void setSlowDownSetpoint(setpoint_t* setpoint)
{

  setpoint->mode.x = modeVelocity;
  setpoint->velocity.x = 0.0f;
  
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.y = 0.0f;

  setpoint->mode.z = modeAbs;
  setpoint->position.z = HOLD_ALTITUDE;  
  
  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = 0.0f;
    
}

void appMain() {

  // DEBUG_PRINT("Waiting for activation ...\n");

  // //  Extract the log ID for the accelerometer (z)
  // logVarId_t idAccZ  = logGetVarId("acc", "z");
  
  // logVarId_t idPx   = logGetVarId("stateEstimate", "x");
  // logVarId_t idPy   = logGetVarId("stateEstimate", "y");
  // logVarId_t idPz   = logGetVarId("stateEstimate", "z");
  // logVarId_t idYaw  = logGetVarId("stateEstimate", "yaw");

  // float acc_z;

  // float px = 0;
  // float py = 0;
  // float pz = 0;
  // float yaw = 0;

  // //  Initialize the FSM with a state
  // StateType state = NORMAL;

  appSetup();

  //  Declare a setpoint variable
  setpoint_t  setpoint;

  //  Declare a variable to hold the start times
  //  You HAVE to init this variable or the compiler
  //  will scream at you for some reason....
  TickType_t startTickCount = xTaskGetTickCount();

  while(1) {

    // Iterate the FSM every 10 milliseconds
    vTaskDelay(M2T(10));

    pullStates();
    // acc_z = logGetFloat(idAccZ);

    switch (state)
    {
    case NORMAL:
      
      //  The precise threshold to determine impact/bouncing
      //  is currently known through experiment
      if(acc_z > ACC_Z_THRESHOLD)
      {
        state = RECOVER;
        // startTickCount = xTaskGetTickCount();
        DEBUG_PRINT("Crashed!\n");
      }
      break;
    
    case RECOVER:
      setRecoverSetpoint(&setpoint, ROLL_RECOVER, PITCH_RECOVER, YAW_RECOVER, THRUST_RECOVER);

      // TODO:  What should be the setpoint priority?
      // commanderSetSetpoint compares the current setpoint with the incoming one.
      // If the incoming priority is >= the current one, the incoming setpoint
      // OVERWRITES the current setpoint. COMMANDER_PRIORITY_CRTP should be enough
      // since the user commands RPYT setpoints during ballistic phase with 
      // COMMANDER_PRIORITY_CRTP priority level.
      commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_CRTP);

      struct quat quat_curr = mkquat(qx, qy, qz, qw);
      struct vec a_z_des = mkvec(0.0f, 0.0f, 1.0f);
      struct vec a_z_curr = qvrot(quat_curr, a_z_des);
      float alpha = degrees(acosf(vdot(a_z_des, a_z_curr)));
      
      float omega_xy_mag = vmag(mkvec(wx, wy, 0.0f));
      // if(xTaskGetTickCount()-startTickCount >= RECOVER_TIME_SPAN)
      if(fabsf(alpha) < ALPHA_THRESHOLD && fabsf(omega_xy_mag) < OMEGA_THRESHOLD)
      {

        // // The first solution
        // state = NORMAL;

        // //  You need to call commanderRelaxPriority() to avoid the
        // //  supervisor watchdog from locking the quadrotor.
        // commanderRelaxPriority();

        // // The second solution
        // state = HOLD;
        // commanderRelaxPriority();
        // startTickCount = xTaskGetTickCount();

        // px = logGetFloat(idPx);
        // py = logGetFloat(idPy);
        // pz = logGetFloat(idPz);
    
        // yaw = logGetFloat(idYaw);

        state = SLOWDOWN;
        commanderRelaxPriority();
        // startTickCount = xTaskGetTickCount();
      }
      break;
    
    case HOLD:
      setHoldSetpoint(&setpoint, px_hold, py_hold, pz_hold, yaw_hold);
      commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_CRTP);
      if(xTaskGetTickCount()-startTickCount >= HOLD_TIME_SPAN)
      {
        state = NORMAL;
        commanderRelaxPriority();
      }
      break;

    case SLOWDOWN:
      setSlowDownSetpoint(&setpoint);
      commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_CRTP);
      // if(xTaskGetTickCount()-startTickCount >= SLOWDOWN_TIME_SPAN)
      float v_xyz_mag = vmag(mkvec(vx, vy, vz));
      if(fabsf(v_xyz_mag) < V_XY_THRESHOLD && fabsf(pz-HOLD_ALTITUDE) < 0.1f)
      {
        state = HOLD;
        commanderRelaxPriority();
        startTickCount = xTaskGetTickCount();

        px_hold = px;
        py_hold = py;
        pz_hold = pz;
        yaw_hold = yaw;
      }      
      break;
    default:
      break;
    }
    // DEBUG_PRINT("Hello from my FSM!\n");
  }
}


LOG_GROUP_START(my_fsm)

LOG_ADD(LOG_UINT8, curr_state, &state)

LOG_GROUP_STOP(my_fsm)