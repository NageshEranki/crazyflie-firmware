
#include "platform_defaults.h"
#include "stabilizer_types.h"
#include "power_distribution.h"

#include "attitude_controller.h"
#include "position_controller.h"
#include "controller_pid.h"
#include "pid.h"

#include "log.h"
#include "param.h"
#include "math3d.h"

// Include FSM header to access state
#include "../../examples/app_my_fsm/src/my_fsm.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)
// #define PITCH_KP              150.0
// #define PITCH_KD              90.0
// #define ROLL_KP               150.0
// #define ROLL_KD               90.0

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;


// Variables for the second attitude controller
// To be used for manual or free-fall flight
static bool isUsingFreefallController = false;


PidObject pidFreeFallRoll = {
  .kp =  ROLL_KP,
  .ki = 0.0f,
  .kd = ROLL_KD,
  .kff = 0.0f,
  .outputLimit = UINT16_MAX/4
};

PidObject pidFreeFallPitch = {
  .kp =  PITCH_KP,
  .ki = 0.0f,
  .kd = PITCH_KD,
  .kff = 0.0f,
  .outputLimit = UINT16_MAX/4
};

static PidObject pidFreeFallYaw = {
  .kp =  125.0f,
  .ki = 0.0f,
  .kd = 0.1f,
  .kff = 0.0f,
};

static void freefallControllerInit(const float updateDt)
{
  pidInit(&pidFreeFallRoll, 0, pidFreeFallRoll.kp, pidFreeFallRoll.ki, pidFreeFallRoll.kd, pidFreeFallRoll.kff, 
        ATTITUDE_UPDATE_DT, ATTITUDE_RATE, ATTITUDE_ROLL_RATE_LPF_CUTOFF_FREQ, true);


  pidInit(&pidFreeFallPitch, 0, pidFreeFallPitch.kp, pidFreeFallPitch.ki, pidFreeFallPitch.kd, pidFreeFallPitch.kff, 
        ATTITUDE_UPDATE_DT, ATTITUDE_RATE, ATTITUDE_PITCH_RATE_LPF_CUTOFF_FREQ, true);


  pidInit(&pidFreeFallYaw, 0, pidFreeFallYaw.kp, pidFreeFallYaw.ki, pidFreeFallYaw.kd, pidFreeFallYaw.kff,
        ATTITUDE_UPDATE_DT, ATTITUDE_RATE, ATTITUDE_YAW_RATE_LPF_CUTOFF_FREQ, true);          


}

void controllerPidInit(void)
{
  attitudeControllerInit(ATTITUDE_UPDATE_DT);
  freefallControllerInit(ATTITUDE_UPDATE_DT);
  positionControllerInit();
}

bool controllerPidTest(void)
{
  bool pass = true;

  pass &= attitudeControllerTest();

  return pass;
}

static float capAngle(float angle) {
  float result = angle;

  while (result > 180.0f) {
    result -= 360.0f;
  }

  while (result < -180.0f) {
    result += 360.0f;
  }

  return result;
}

void freefallResetAttitudeController(const float rollActual, const float pitchActual, const float yawActual)
{
  pidReset(&pidFreeFallRoll,  rollActual);
  pidReset(&pidFreeFallPitch, pitchActual);
  pidReset(&pidFreeFallYaw,   yawActual);
}

void freefallAttitudeController(const float rollActual, const float pitchActual, const float yawActual,
                                const float rollDes, const float pitchDes, const float yawDes,
                                control_t* control)
{
  pidSetDesired(&pidFreeFallRoll,   rollDes);
  pidSetDesired(&pidFreeFallPitch,  pitchDes);
  pidSetDesired(&pidFreeFallYaw,    yawDes);

  control->roll =   pidUpdate(&pidFreeFallRoll,   rollActual,   false);
  control->pitch =  pidUpdate(&pidFreeFallPitch,  pitchActual,  false);
  control->yaw =    pidUpdate(&pidFreeFallYaw,    yawActual,    true);
  
  // control->thrust is set before this function is called
}

void controllerPid(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *stateEstimate,
                                         const stabilizerStep_t stabilizerStep)
{
  control->controlMode = controlModeLegacy;

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
    // Rate-controled YAW is moving YAW angle setpoint
    if (setpoint->mode.yaw == modeVelocity) {
      attitudeDesired.yaw = capAngle(attitudeDesired.yaw + setpoint->attitudeRate.yaw * ATTITUDE_UPDATE_DT);

      float yawMaxDelta = attitudeControllerGetYawMaxDelta();
      if (yawMaxDelta != 0.0f)
      {
      float delta = capAngle(attitudeDesired.yaw-stateEstimate->attitude.yaw);
      // keep the yaw setpoint within +/- yawMaxDelta from the current yaw
        if (delta > yawMaxDelta)
        {
          attitudeDesired.yaw = stateEstimate->attitude.yaw + yawMaxDelta;
        }
        else if (delta < -yawMaxDelta)
        {
          attitudeDesired.yaw = stateEstimate->attitude.yaw - yawMaxDelta;
        }
      }
    } else if (setpoint->mode.yaw == modeAbs) {
      attitudeDesired.yaw = setpoint->attitude.yaw;
    } else if (setpoint->mode.quat == modeAbs) {
      struct quat setpoint_quat = mkquat(setpoint->attitudeQuaternion.x, setpoint->attitudeQuaternion.y, setpoint->attitudeQuaternion.z, setpoint->attitudeQuaternion.w);
      struct vec rpy = quat2rpy(setpoint_quat);
      attitudeDesired.yaw = degrees(rpy.z);
    }

    attitudeDesired.yaw = capAngle(attitudeDesired.yaw);
  }

  if (RATE_DO_EXECUTE(POSITION_RATE, stabilizerStep)) {
    positionController(&actuatorThrust, &attitudeDesired, setpoint, stateEstimate);
  }

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
    // Switch between manual and automatic position control
    if (setpoint->mode.z == modeDisable) {
      actuatorThrust = setpoint->thrust;
    }
    if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
      attitudeDesired.roll = setpoint->attitude.roll;
      attitudeDesired.pitch = setpoint->attitude.pitch;

    }

    attitudeControllerCorrectAttitudePID(stateEstimate->attitude.roll, stateEstimate->attitude.pitch, stateEstimate->attitude.yaw,
                                attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
                                &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);

    // For roll and pitch, if velocity mode, overwrite rateDesired with the setpoint
    // value. Also reset the PID to avoid error buildup, which can lead to unstable
    // behavior if level mode is engaged later
    if (setpoint->mode.roll == modeVelocity) {
      rateDesired.roll = setpoint->attitudeRate.roll;
      attitudeControllerResetRollAttitudePID(stateEstimate->attitude.roll);
    }
    if (setpoint->mode.pitch == modeVelocity) {
      rateDesired.pitch = setpoint->attitudeRate.pitch;
      attitudeControllerResetPitchAttitudePID(stateEstimate->attitude.pitch);
    }

    // TODO: Investigate possibility to subtract gyro drift.
    attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
                             rateDesired.roll, rateDesired.pitch, rateDesired.yaw);

    attitudeControllerGetActuatorOutput(&control->roll,
                                        &control->pitch,
                                        &control->yaw);

    // TODO: If the operator wants manual control of RPYT, you MUST
    // override the stock attitude and attitude-rate PID control inputs
    // AND you may want to use tilt-prioritzied control allocation
    if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable)
    {
      // TODO: Try the following sometime
      // When switching to attitude control, you MUST prioritize pitch and roll torque
      // commands over the collective thrust and yaw-torque.
      use_tilt_priority = true;

      // Reset if switching to second attitude control in this iteration
      if(!isUsingFreefallController)
      {
        freefallResetAttitudeController(stateEstimate->attitude.roll, stateEstimate->attitude.pitch, stateEstimate->attitude.yaw);
        isUsingFreefallController = true;
      }
      
      freefallAttitudeController(
        stateEstimate->attitude.roll, stateEstimate->attitude.pitch, stateEstimate->attitude.yaw,
        setpoint->attitude.roll, setpoint->attitude.pitch, setpoint->attitude.yaw,
        control);

    }else
    {
      // Enable tilt-priority allocation for SLOWDOWN state
      use_tilt_priority = (state != NORMAL);

      // Reset if switching back to stock controller
      if (isUsingFreefallController) {
          attitudeControllerResetAllPID(stateEstimate->attitude.roll, stateEstimate->attitude.pitch, stateEstimate->attitude.yaw);
          positionControllerResetAllPID(stateEstimate->position.x, stateEstimate->position.y, stateEstimate->position.z);
          isUsingFreefallController = false;
      }
    }                                        

    control->yaw = -control->yaw;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;
    r_roll = radians(sensors->gyro.x);
    r_pitch = -radians(sensors->gyro.y);
    r_yaw = radians(sensors->gyro.z);
    accelz = sensors->acc.z;
  }

  control->thrust = actuatorThrust;

  if (control->thrust == 0)
  {
    control->thrust = 0;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;

    attitudeControllerResetAllPID(stateEstimate->attitude.roll, stateEstimate->attitude.pitch, stateEstimate->attitude.yaw);
    positionControllerResetAllPID(stateEstimate->position.x, stateEstimate->position.y, stateEstimate->position.z);

    // Reset the calculated YAW angle for rate control
    attitudeDesired.yaw = stateEstimate->attitude.yaw;
  }
}

/**
 * Logging variables for the command and reference signals for the
 * altitude PID controller
 */
LOG_GROUP_START(controller)
/**
 * @brief Thrust command
 */
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
/**
 * @brief Roll command
 */
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
/**
 * @brief Pitch command
 */
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
/**
 * @brief yaw command
 */
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
/**
 * @brief Gyro roll measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
/**
 * @brief Gyro pitch measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
/**
 * @brief Yaw  measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
/**
 * @brief Acceleration in the zaxis in G-force
 */
LOG_ADD(LOG_FLOAT, accelz, &accelz)
/**
 * @brief Thrust command without (tilt)compensation
 */
LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
/**
 * @brief Desired roll setpoint
 */
LOG_ADD(LOG_FLOAT, roll,      &attitudeDesired.roll)
/**
 * @brief Desired pitch setpoint
 */
LOG_ADD(LOG_FLOAT, pitch,     &attitudeDesired.pitch)
/**
 * @brief Desired yaw setpoint
 */
LOG_ADD(LOG_FLOAT, yaw,       &attitudeDesired.yaw)
/**
 * @brief Desired roll rate setpoint
 */
LOG_ADD(LOG_FLOAT, rollRate,  &rateDesired.roll)
/**
 * @brief Desired pitch rate setpoint
 */
LOG_ADD(LOG_FLOAT, pitchRate, &rateDesired.pitch)
/**
 * @brief Desired yaw rate setpoint
 */
LOG_ADD(LOG_FLOAT, yawRate,   &rateDesired.yaw)
LOG_GROUP_STOP(controller)

LOG_GROUP_START(pid_freefall)
/**
 * @brief Proportional output pitch
 */
LOG_ADD(LOG_FLOAT, pitch_outP, &pidFreeFallPitch.outP)
/**
 * @brief Derivative output pitch
 */
LOG_ADD(LOG_FLOAT, pitch_outD, &pidFreeFallPitch.outD)
/**
 * @brief Proportional output roll
 */
LOG_ADD(LOG_FLOAT, roll_outP, &pidFreeFallRoll.outP)
/**
 * @brief Derivative output roll
 */
LOG_ADD(LOG_FLOAT, roll_outD, &pidFreeFallRoll.outD)
/**
 * @brief Proportional output yaw
 */
LOG_ADD(LOG_FLOAT, yaw_outP, &pidFreeFallYaw.outP)
/**
 * @brief Derivative output yaw
 */
LOG_ADD(LOG_FLOAT, yaw_outD, &pidFreeFallYaw.outD)
LOG_GROUP_STOP(pid_freefall)

PARAM_GROUP_START(pid2Param)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT,  pitchKp, &pidFreeFallPitch.kp)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT,  pitchKd, &pidFreeFallPitch.kd)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT,  rollKp, &pidFreeFallRoll.kp)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT,  rollKd, &pidFreeFallRoll.kd)
PARAM_GROUP_STOP(pid2Param)