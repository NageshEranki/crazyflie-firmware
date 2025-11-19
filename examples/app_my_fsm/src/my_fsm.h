#ifndef __FINITE_STATE_MACHINE__
#define __FINITE_STATE_MACHINE__

//  Use a custom flag to control the conditional compilation.
//  You will share global variables defined here with 'core'
//  modules (such as crtp_commander.c). Without conditional 
//  compilation, you cannot build the firmware from the root 
//  directory as the global variable references (e.g. 'state')
//  will be undefined.

//  UPDATE: This does NOT work properly
//  #define MY_FSM_ENABLED  1

#include <stdint.h>

//  Copied these thurst limits from crtp_commander_rpyt.c
#define MIN_THRUST  1000
#define MAX_THRUST  60000

//  Define some parameters/thresholds
static const float ACC_Z_IMPACT_THRESH = 2.5f;
static const float ACC_Z_REC_THRESH = 0.1f;
static const float ROLL_RECOVER = 0.0f;
static const float PITCH_RECOVER = 0.0f;
static const float YAW_RECOVER = 0.0f;
static const float ALPHA_THRESHOLD = 7.5f;
static const float OMEGA_THRESHOLD = 10.0f;
static const float V_XY_THRESHOLD = 0.1f;
static const uint16_t THRUST_RECOVER = 44000;
static const unsigned int RECOVER_TIME_SPAN = 1000;
static const unsigned int HOLD_TIME_SPAN = 5000;
static const unsigned int SLOWDOWN_TIME_SPAN = 5000;
static const float HOLD_ALTITUDE = 0.8f;

//  Define the list of states
typedef enum
{
  NORMAL,
  RECOVER,
  HOLD,
  SLOWDOWN,
  IMPACT
} StateType;

extern StateType state;

#endif