/*
 * File: usartData.h
 *
 */

#ifndef _USART_DATA_H
#define _USART_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef RTWTYPES_H

typedef signed char int8_T;
typedef unsigned char uint8_T;
typedef short int16_T;
typedef unsigned short uint16_T;
typedef int int32_T;
typedef unsigned int uint32_T;
typedef long long int64_T;
typedef unsigned long long uint64_T;
typedef float real32_T;
typedef double real64_T;
typedef unsigned char boolean_T;

#endif

/* --------------------- EXTU -------------------------------*/
#ifdef USE_EXTU
#define EXTU_START_FRAME      0xABCD

/* External inputs (root inport signals with auto storage) */
typedef struct {
  boolean_T b_motEna;                  /* '<Root>/b_motEna' */
  uint8_T z_ctrlModReq;                /* '<Root>/z_ctrlModReq' */
  int16_T r_inpTgt;                    /* '<Root>/r_inpTgt' */
  uint8_T b_hallA;                     /* '<Root>/b_hallA ' */
  uint8_T b_hallB;                     /* '<Root>/b_hallB' */
  uint8_T b_hallC;                     /* '<Root>/b_hallC' */
  int16_T i_phaAB;                     /* '<Root>/i_phaAB' */
  int16_T i_phaBC;                     /* '<Root>/i_phaBC' */
  int16_T i_DCLink;                    /* '<Root>/i_DCLink' */
  int16_T a_mechAngle;                 /* '<Root>/a_mechAngle' */
} ExtU;

uint16_T ExtU_calcChecksum(ExtU *ptr);

#endif  // USE_EXTU

/* --------------------- EXTY -------------------------------*/
#ifdef USE_EXTY
#define EXTY_START_FRAME      0xABCD

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  int16_T DC_phaA;                     /* '<Root>/DC_phaA' */
  int16_T DC_phaB;                     /* '<Root>/DC_phaB' */
  int16_T DC_phaC;                     /* '<Root>/DC_phaC' */
  uint8_T z_errCode;                   /* '<Root>/z_errCode' */
  int16_T n_mot;                       /* '<Root>/n_mot' */
  int16_T a_elecAngle;                 /* '<Root>/a_elecAngle' */
  int16_T iq;                          /* '<Root>/iq' */
  int16_T id;                          /* '<Root>/id' */
  uint32_T timestamp;
} ExtY;

uint16_T ExtY_calcChecksum(ExtY *ptr);

#endif  // USE_EXTY

/* --------------------- MAIN_STATUS ------------------------------*/
#ifdef USE_MAIN_STATUS
#define MAIN_STATUS_START_FRAME      0xABCD

typedef struct{
  uint16_T  start;
  int16_T   cmd1;
  int16_T   cmd2;
  int16_T   speedR_meas;
  int16_T   speedL_meas;
  int16_T   batVoltage;
  int16_T   boardTemp;
  uint16_T  cmdLed;
#ifdef USE_EXTU
  ExtU      rightExtU;
  ExtU      leftExtU;
#endif
#ifdef USE_EXTY
  ExtY      rightExtY;
  ExtY      leftExtY;
#endif
  uint16_T  checksum;
} MainStatus;

uint16_T MainStatus_calcChecksum(MainStatus *ptr);
void MainStatus_fillExtraData(MainStatus *ptr, void *extURight, void *extULeft, void *extYRight, void *extYLeft);

#endif // USE_MAIN_STATUS


/* --------------------- MOTOR_CONTROL ------------------------------*/
#ifdef USE_MOTOR_CONTROL
#define MOTOR_CONTROL_START_FRAME      0xABCD

typedef struct{
  uint16_T  start;
  int16_T   steer;
  int16_T   speed;
  uint16_T  checksum;
} MotorControl;

uint16_T MotorControl_calcChecksum(MotorControl *ptr);

#endif // USE_MOTOR_CONTROL

/* --------------------- POSITION_STATUS ------------------------------*/
#ifdef USE_POSITION_STATUS
#define POSITION_STATUS_START_FRAME      0xABCD

typedef struct{
  uint16_T  start;
  int16_T   temp;      // Temperature
  int16_T   roll;      // Angle
  int16_T   pitch;     // Angle
  int16_T   yaw;       // Angle
  int16_T   accX;      // Accelleration
  int16_T   accY;      // Accelleration
  int16_T   accZ;      // Accelleration
  uint32_T  timestamp;
  uint16_T  checksum;
} PositionStatus;

uint16_T PositionStatus_calcChecksum(PositionStatus *ptr);

#endif // USE_POSITION_STATUS

#ifdef __cplusplus
}
#endif

#endif