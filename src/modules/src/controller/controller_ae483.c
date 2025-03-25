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


// Observer switches
static bool use_observer = false;
static bool reset_observer = false;

static uint32_t portcount = 0;


// Setpoint
static float p_x_des = 0.0f;
static float p_y_des = 0.0f;
static float p_z_des = 0.0f;


// Control inputs for position subsystem a.k.a attitude setpoints
static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;

// Init encoder objects
static encoder_t x_encoder = {0.0f, 0.0f, 1.0f, 1.02f, 15, false, 0};
static encoder_t y_encoder = {0.0f, 0.0f, 1.0f, 1.02f, 15, false, 0};
static encoder_t z_encoder = {0.0f, 0.0f, 1.0f, 1.02f, 15, false, 0};


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

void decoder(const uint8_t qk, encoder_t* this)
{
  // If the codeword is 'MAX_VALUE', we are zooming out.
  // DO NOT modify the state estimate.
  if(qk == (this->Ndiv*this->Ndiv))
  {
    // DEBUG_PRINT("State overflowed\n");
    this->hasOverflowed = true;
    return;
  }
  else{
    // Unpack the received codeword to update state-estimates
    uint8_t qk_p_z, qk_v_z, qk_copy;

    qk_copy = qk;
    qk_p_z = qk_copy % this->Ndiv;
    qk_copy = qk_copy / this->Ndiv;
    qk_v_z = qk_copy % this->Ndiv;
    
    float delta = 2.0f * this->E0 / this->Ndiv;

    this->p_hat = -this->E0 + this->p_hat + delta/2.0f + qk_p_z*delta;
    this->v_hat = -this->E0 + this->v_hat + delta/2.0f + qk_v_z*delta;

    this->hasOverflowed = false;
    return;
  }
}

void updateBoundingBox(encoder_t *this)
{
  if(this->hasOverflowed)
  {
    this->E0 = 1.0f;
    for(uint16_t i=0; i<3*this->qk_count; i++)
    {
      this->E0 *= (this->qk_count * this->Lambda);
    }

    this->qk_count += 1;
  }
  else
  {
    this->E0 *= (this->Lambda / this->Ndiv);
    this->qk_count = 0;
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
  
  // Iterate decoders
  decoder(data->qk_x, &x_encoder);
  decoder(data->qk_y, &y_encoder);
  decoder(data->qk_z, &z_encoder);

  //  Update the size of the bounding box.
  // 'updateBoundingBox' modifies the 'E0' member variable
  updateBoundingBox(&x_encoder);
  updateBoundingBox(&y_encoder);
  updateBoundingBox(&z_encoder);

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

      // Reset all decoder state estimates
      x_encoder.p_hat = 0.0f;
      y_encoder.p_hat = 0.0f;
      z_encoder.p_hat = 0.0f;

      x_encoder.v_hat = 0.0f;
      y_encoder.v_hat = 0.0f;
      z_encoder.v_hat = 0.0f;

      reset_observer = false;
    }

    // For the x-position subsystem, we integrate the closed-loop
    // system.
    if(x_encoder.hasOverflowed)
    {
      attitudeDesired.pitch = 0.0f;

      x_encoder.p_hat += ((float)(1.0f/POSITION_RATE) * x_encoder.v_hat);
    }
    else
    {
      attitudeDesired.pitch = -50.0f * (p_x_des - x_encoder.p_hat) + 25.0f * (x_encoder.v_hat);

      x_encoder.p_hat += ((float)(1.0f/POSITION_RATE) * x_encoder.v_hat);
      x_encoder.v_hat += ((float)(1.0f/POSITION_RATE) * (-8.560f*(x_encoder.p_hat-p_x_des) - 4.280f*x_encoder.v_hat));
    }

    // For the y-position subsystem, we integrate the closed-loop
    // system.
    if(y_encoder.hasOverflowed)
    {
      attitudeDesired.roll = 0.0f;

      y_encoder.p_hat += ((float)(1.0f/POSITION_RATE) * y_encoder.v_hat);
    }
    else
    {
      attitudeDesired.roll = -50.0f * (p_y_des - y_encoder.p_hat) + 25.0f * (y_encoder.v_hat);

      y_encoder.p_hat += ((float)(1.0f/POSITION_RATE) * y_encoder.v_hat);
      y_encoder.v_hat += ((float)(1.0f/POSITION_RATE) * (-8.560f*(y_encoder.p_hat-p_y_des) - 4.280f*y_encoder.v_hat));
    }

    // For the z-position subsystem, we integrate the closed-loop
    // system.
    if(z_encoder.hasOverflowed)
    {
      actuatorThrust = 40000.0f;

      z_encoder.p_hat += ((float)(1.0f/POSITION_RATE) * z_encoder.v_hat);
    }
    else
    {
      actuatorThrust = 1000.0f * (85.0f * (p_z_des - z_encoder.p_hat) - 17.0f * (z_encoder.v_hat)) + 40000.0f;

      z_encoder.p_hat += ((float)(1.0f/POSITION_RATE) * z_encoder.v_hat);
      z_encoder.v_hat += ((float)(1.0f/POSITION_RATE) * (-19.8254f*(z_encoder.p_hat-p_z_des) - 5.831f*z_encoder.v_hat));
    }

    // saturate control inputs
    attitudeDesired.pitch = constrain(attitudeDesired.pitch, -pLimit, pLimit);
    attitudeDesired.roll = constrain(attitudeDesired.roll, -rLimit, rLimit);
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

  // // Uncomment if you are running a Processor-in-loop test
  // control->thrust = 0;
  // control->roll = 0;
  // control->pitch = 0;
  // control->yaw = 0;
}

// 1234567890123456789012345678 <-- max total length
// group   .name
LOG_GROUP_START(ae483log)
LOG_ADD(LOG_UINT16,      num_tof,                &tof_count)
LOG_ADD(LOG_UINT16,      num_flow,               &flow_count)
// LOG_ADD(LOG_FLOAT,       p_x_mocap,              &p_x_mocap)
LOG_ADD(LOG_FLOAT,       E0_z,                   &z_encoder.E0)
LOG_ADD(LOG_FLOAT,       E0_y,                   &y_encoder.E0)
LOG_ADD(LOG_FLOAT,       p_x,                    &x_encoder.p_hat)
LOG_ADD(LOG_FLOAT,       p_y,                    &y_encoder.p_hat)
LOG_ADD(LOG_FLOAT,       p_z,                    &z_encoder.p_hat)
LOG_ADD(LOG_FLOAT,       v_x,                    &x_encoder.v_hat)
LOG_ADD(LOG_FLOAT,       v_y,                    &y_encoder.v_hat)
LOG_ADD(LOG_FLOAT,       v_z,                    &z_encoder.v_hat)
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
PARAM_ADD(PARAM_UINT8,     Ndiv_x,                  &x_encoder.Ndiv)
PARAM_ADD(PARAM_FLOAT,     Lambda_x,                &x_encoder.Lambda)
PARAM_ADD(PARAM_UINT8,     Ndiv_y,                  &y_encoder.Ndiv)
PARAM_ADD(PARAM_FLOAT,     Lambda_y,                &y_encoder.Lambda)
PARAM_ADD(PARAM_UINT8,     Ndiv_z,                  &z_encoder.Ndiv)
PARAM_ADD(PARAM_FLOAT,     Lambda_z,                &z_encoder.Lambda)
PARAM_GROUP_STOP(ae483par)
