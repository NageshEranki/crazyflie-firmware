#include "controller_ae483.h"

#include "attitude_controller.h"
#include "position_controller.h"
#include "controller_pid.h"

#include "stabilizer_types.h"
#include "power_distribution.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"

#include "debug.h"

#define DEBUG_MODULE "CONTROLLER AE483"
#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)



// Pitch and roll saturation limits (deg)
static const float rLimit = 20.0f;
static const float pLimit = 20.0f;


// Sensor measurements
// - tof (from the z ranger on the flow deck)
static uint16_t tof_count = 0;
static float tof_distance = 0.0f;
// - flow (from the optical flow sensor on the flow deck)
static uint16_t flow_count = 0;
static float flow_dpixelx = 0.0f;
static float flow_dpixely = 0.0f;
// - Mocap data
static float p_x_mocap = 0.0f;
static float p_y_mocap = 0.0f;
static float p_z_mocap = 0.0f;
static float psi_mocap = 0.0f;
static float theta_mocap = 0.0f;
static float phi_mocap = 0.0f;

// Parameters
static bool use_observer = false;
static bool reset_observer = false;
static float MOCAP_HZ = 100.0f;
static float Lambda = 0.0f;
static unsigned char Ndiv = 0;

// States
// - Position
static float p_x = 0.0f;
static float p_y = 0.0f;
static float p_z = 0.0f;
// - Velocity
static float v_x = 0.0f;
static float v_y = 0.0f;
static float v_z = 0.0f;
// - Velocity (based on mocap, finite differences)
static float v_x_mocap = 0.0f;
static float v_y_mocap = 0.0f;
static float v_z_mocap = 0.0f;
static uint32_t portcount = 0;
// Encoder variables
static float E0 = 1.0f;
// static const float Lambda = 1.020f;
// static const unsigned char Ndiv = 15;
static bool hasOverflowed = false;
static uint16_t qk_count = 0;

// Setpoint
static float p_x_des = 0.0f;
static float p_y_des = 0.0f;
static float p_z_des = 0.0f;


// Control inputs for position subsystem
static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;


void ae483UpdateWithTOF(tofMeasurement_t *tof)
{
  tof_distance = tof->distance;
  tof_count++;
}

void ae483UpdateWithFlow(flowMeasurement_t *flow)
{
  flow_dpixelx = flow->dpixelx;
  flow_dpixely = flow->dpixely;
  flow_count++;
}

void ae483UpdateWithDistance(distanceMeasurement_t *meas)
{
  // If you have a loco positioning deck, this function will be called
  // each time a distance measurement is available. You will have to write
  // code to handle these measurements. These data are available:
  //
  //  meas->anchorId  uint8_t   id of anchor with respect to which distance was measured
  //  meas->x         float     x position of this anchor
  //  meas->y         float     y position of this anchor
  //  meas->z         float     z position of this anchor
  //  meas->distance  float     the measured distance
}

void ae483UpdateWithPosition(positionMeasurement_t *meas)
{
  // This function will be called each time you send an external position
  // measurement (x, y, z) from the client, e.g., from a motion capture system.
  // You will have to write code to handle these measurements. These data are
  // available:
  //
  //  meas->x         float     x component of external position measurement
  //  meas->y         float     y component of external position measurement
  //  meas->z         float     z component of external position measurement
}

void ae483UpdateWithPose(poseMeasurement_t *meas)
{
  // This function will be called each time you send an external "pose" measurement
  // (position as x, y, z and orientation as quaternion) from the client, e.g., from
  // a motion capture system. You will have to write code to handle these measurements.
  // These data are available:
  //
  //  meas->x         float     x component of external position measurement
  //  meas->y         float     y component of external position measurement
  //  meas->z         float     z component of external position measurement
  //  meas->quat.x    float     x component of quaternion from external orientation measurement
  //  meas->quat.y    float     y component of quaternion from external orientation measurement
  //  meas->quat.z    float     z component of quaternion from external orientation measurement
  //  meas->quat.w    float     w component of quaternion from external orientation measurement

}

bool decoder(const int16_t qk_p_z, const int16_t qk_v_z)
{
  // If the codeword is '-1', we are zooming out.
  // DO NOT modify the state estimate.
  if(qk_p_z == -1)
  {
    // Do nothing.
    // DEBUG_PRINT("State overflowed\n");

    return true;
  }
  
  else{
    
    float delta = 2.0f * E0 / Ndiv;

    p_z_mocap = -E0 + p_z + delta/2.0f + qk_p_z*delta;
    v_z_mocap = -E0 + v_z + delta/2.0f + qk_v_z*delta;

    p_z = p_z_mocap;
    v_z = v_z_mocap;

    return false;
  }
}

void updateBoundingBox(const bool hasOverflowed)
{
  if(hasOverflowed)
  {
    E0 = 1.0f;
    for(int i=0; i<3*qk_count; i++)
    {
      E0 *= (qk_count * Lambda);
    }

    qk_count += 1;
  }
  else
  {
    E0 *= (Lambda / Ndiv);
    qk_count = 0;
  }
}

void ae483UpdateWithData(const struct AE483Data* data)
{
  // This function will be called each time AE483-specific data are sent
  // from the client to the drone. You will have to write code to handle
  // these data. For the example AE483Data struct, these data are:
  //
  //  data->x         float
  //  data->y         float
  //  data->z         float
  //
  // Exactly what "x", "y", and "z" mean in this context is up to you.
  
  // Position
  p_x_mocap = data->p_x;
  p_y_mocap = data->p_y;
  // p_z_mocap = data->p_z;

  // Velocity
  v_x_mocap = data->v_x;
  v_y_mocap = data->v_y;
  // v_z_mocap = data->v_z;

  // Decoder for z-position subsystem
  hasOverflowed = decoder(data->qk_p_z, data->qk_v_z);

  // Update the size of the bounding box.
  // E0 is modified IN-PLACE
  updateBoundingBox(hasOverflowed);

}


void controllerAE483Init(void)
{

  DEBUG_PRINT("Called 'Init' on my controller! :)\n");

  // Forward init calls to default PID controller
  attitudeControllerInit(ATTITUDE_UPDATE_DT);
  // positionControllerInit();
}

bool controllerAE483Test(void)
{
  // Forward call to attitude control test
  return attitudeControllerTest();
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

void controllerAE483(control_t *control,
                     const setpoint_t *setpoint,
                     const sensorData_t *sensors,
                     const state_t *state,
                     const stabilizerStep_t stabilizerStep)
{
  control->controlMode = controlModeLegacy;

  p_x_des = setpoint->position.x;
  p_y_des = setpoint->position.y;
  p_z_des = setpoint->position.z;

  if (RATE_DO_EXECUTE(POSITION_RATE, stabilizerStep)) {
    
    // FIXME: Insert custom observer here
    if(reset_observer)
    {
      p_x = 0.0f;
      p_y = 0.0f;
      p_z = 0.0f;
      v_x = 0.0f;
      v_y = 0.0f;
      v_z = 0.0f;

      p_x_mocap = 0.0f;
      p_y_mocap = 0.0f;
      p_z_mocap = 0.0f;
      v_x_mocap = 0.0f;
      v_y_mocap = 0.0f;
      v_z_mocap = 0.0f;

      reset_observer = false;
    }
    // - Position
    p_x = p_x_mocap;
    p_y = p_y_mocap;

    // - Velocity
    v_x = v_x_mocap;
    v_y = v_y_mocap;

    // Feedback for position subsystem
    attitudeDesired.pitch = -50.0f * (p_x_des - p_x) + 25.0f * (v_x);
    attitudeDesired.roll = -50.0f * (p_y_des - p_y) + 25.0f * (v_y);


    // saturate control inputs
    attitudeDesired.pitch = constrain(attitudeDesired.pitch, -pLimit, pLimit);
    attitudeDesired.roll = constrain(attitudeDesired.roll, -rLimit, rLimit);

    // For the z-position subsystem, we integrate the closed-loop
    // system.
    // if(setpoint->mode.z != modeDisable)
    if(hasOverflowed)
    {
      actuatorThrust = 40000.0f;

      p_z += ((float)(1.0f/POSITION_RATE) * v_z);
    }
    else
    {
      actuatorThrust = 1000.0f * (85.0f * (p_z_des - p_z) - 17.0f * (v_z)) + 40000.0f;

      p_z += ((float)(1.0f/POSITION_RATE) * v_z);
      v_z += ((float)(1.0f/POSITION_RATE) * (-19.8254f*(p_z-p_z_des) - 5.831f*v_z));
    }

    actuatorThrust = constrain(actuatorThrust, 0, UINT16_MAX);

  }

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {

    if (setpoint->mode.yaw == modeAbs) {
      attitudeDesired.yaw = setpoint->attitude.yaw;
    } 

    attitudeDesired.yaw = capAngle(attitudeDesired.yaw);

    // Switch between manual and automatic position control
    if (setpoint->mode.z == modeDisable) {

      actuatorThrust = 0.0f;

    }

    // Attitude PID:
    // Input:  Desired attitude
    // Output: Desired attitude rates
    attitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
                                attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
                                &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);


    // Attitude rate PID:
    // Input:   Desired attitude rates
    // Output:  Roll/Pitch/Yaw body torques
    // Q). Where are the output variables from PID?
    attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
                             rateDesired.roll, rateDesired.pitch, rateDesired.yaw);

    // Convert torques to actuator ouputs
    attitudeControllerGetActuatorOutput(&control->roll,
                                        &control->pitch,
                                        &control->yaw);

    control->yaw = -control->yaw;
  }

  control->thrust = actuatorThrust;

  if (control->thrust == 0)
  {
    control->thrust = 0;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    attitudeControllerResetAllPID();

    // Reset the calculated YAW angle for rate control
    attitudeDesired.yaw = state->attitude.yaw;
  }
}

// 1234567890123456789012345678 <-- max total length
// group   .name
LOG_GROUP_START(ae483log)
LOG_ADD(LOG_UINT16,      num_tof,                &tof_count)
LOG_ADD(LOG_UINT16,      num_flow,               &flow_count)
LOG_ADD(LOG_FLOAT,       p_x_mocap,              &p_x_mocap)
LOG_ADD(LOG_FLOAT,       p_y_mocap,              &p_y_mocap)
LOG_ADD(LOG_FLOAT,       p_z_mocap,              &p_z_mocap)
LOG_ADD(LOG_FLOAT,       v_x_mocap,              &v_x_mocap)
LOG_ADD(LOG_FLOAT,       v_y_mocap,              &v_y_mocap)
LOG_ADD(LOG_FLOAT,       v_z_mocap,              &v_z_mocap)
LOG_ADD(LOG_FLOAT,       psi_mocap,              &psi_mocap)
LOG_ADD(LOG_FLOAT,       theta_mocap,            &theta_mocap)
LOG_ADD(LOG_FLOAT,       phi_mocap,              &phi_mocap)
LOG_ADD(LOG_FLOAT,       E0,                     &E0)
LOG_ADD(LOG_FLOAT,       p_x,                    &p_x)
LOG_ADD(LOG_FLOAT,       p_y,                    &p_y)
LOG_ADD(LOG_FLOAT,       p_z,                    &p_z)
LOG_ADD(LOG_FLOAT,       v_x,                    &v_x)
LOG_ADD(LOG_FLOAT,       v_y,                    &v_y)
LOG_ADD(LOG_FLOAT,       v_z,                    &v_z)
LOG_ADD(LOG_FLOAT,       p_x_des,                &p_x_des)
LOG_ADD(LOG_FLOAT,       p_y_des,                &p_y_des)
LOG_ADD(LOG_FLOAT,       p_z_des,                &p_z_des)
LOG_ADD(LOG_UINT32,      portcount,              &portcount)
LOG_GROUP_STOP(ae483log)

// 1234567890123456789012345678 <-- max total length
// group   .name
PARAM_GROUP_START(ae483par)
PARAM_ADD(PARAM_UINT8,     use_observer,            &use_observer)
PARAM_ADD(PARAM_UINT8,     reset_observer,          &reset_observer)
PARAM_ADD(PARAM_UINT8,     Ndiv,                    &Ndiv)
PARAM_ADD(PARAM_FLOAT,     MOCAP_HZ,                &MOCAP_HZ)
PARAM_ADD(PARAM_FLOAT,     Lambda,                  &Lambda)
PARAM_GROUP_STOP(ae483par)
