#ifndef __CONTROLLER_AE483_H__
#define __CONTROLLER_AE483_H__

#include "stabilizer_types.h"


// Custom struct to hold some coding scheme parameters.
// x, y & z position subsystems are modelled as 3 independent
// double-integrator systems so this struct will provide some
// modularity in code.
typedef struct {

  float p_hat;
  float v_hat;
  float E0;
  float Lambda;
  uint8_t Ndiv;
  bool hasOverflowed;
  uint16_t qk_count;

} encoder_t;

typedef struct encoded_drone_data_t
{
  uint8_t qk_x;
  uint8_t qk_y;
  uint8_t qk_z;
} encoded_drone_data_t; // 3 bytes

// An example struct to hold AE483-specific data sent from client to drone
struct AE483Data
{
  
  // Idempotency token
  uint8_t idem_token;
  
  // Encoded state measurement for 9 drones
  encoded_drone_data_t encoded_drone_data[9];

} __attribute__((packed));

void controllerAE483Init(void);
bool controllerAE483Test(void);
void controllerAE483(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep);

// Functions to receive measurements
void ae483UpdateWithTOF(tofMeasurement_t *tof);
void ae483UpdateWithFlow(flowMeasurement_t *flow);
void ae483UpdateWithDistance(distanceMeasurement_t *meas);
void ae483UpdateWithPosition(positionMeasurement_t *meas);
void ae483UpdateWithPose(poseMeasurement_t *meas);

// Functions to receive AE483-specific data sent from client to drone
void ae483UpdateWithData(const struct AE483Data* data);


#endif //__CONTROLLER_AE483_H__
