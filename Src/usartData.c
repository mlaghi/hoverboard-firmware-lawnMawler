/*
 * File: usartData.c
 */

#include "usartData.h"
#include <string.h>

#ifdef USE_EXTU
uint16_T ExtU_calcChecksum(ExtU *ptr) {
  uint16_T checksum = ptr->b_motEna ^ ptr->z_ctrlModReq ^ ptr->r_inpTgt ^ ptr->b_hallA
    ^ ptr->b_hallB ^ ptr->b_hallC ^ ptr->i_phaAB ^ ptr->i_phaBC ^ ptr->i_DCLink ^ ptr->a_mechAngle;
    return checksum;
}
#endif // USE_EXTU

#ifdef USE_EXTY
uint16_T ExtY_calcChecksum(ExtY *ptr) {
  uint16_T checksum = ptr->DC_phaA ^ ptr->DC_phaB ^ ptr->DC_phaC ^ ptr->z_errCode
    ^ ptr->n_mot ^ ptr->a_elecAngle ^ ptr->iq ^ ptr->id ^ ptr->timestamp;
    return checksum;
}
#endif // USE_EXTY

#ifdef USE_MAIN_STATUS
uint16_T MainStatus_calcChecksum(MainStatus *ptr) {
    uint16_T checksum = (uint16_T)(ptr->start ^ ptr->cmd1 ^ ptr->cmd2 ^ ptr->speedR_meas ^ ptr->speedL_meas 
                        ^ ptr->batVoltage ^ ptr->boardTemp ^ ptr->cmdLed);
#ifdef USE_EXTU
    checksum ^= ExtU_calcChecksum(&(ptr->rightExtU)) ^ ExtU_calcChecksum(&(ptr->leftExtU));
#endif
#ifdef USE_EXTY
    checksum ^= ExtY_calcChecksum(&(ptr->rightExtY)) ^ ExtY_calcChecksum(&(ptr->rightExtY));
#endif
#ifdef USE_POSITION_STATUS
    checksum ^= PositionStatus_calcChecksum(&(ptr->position));
#endif
    return checksum;
}

void MainStatus_fillExtraData(MainStatus *ptr, void *extURight, void *extULeft, void *extYRight, void *extYLeft, void *position) {
#ifdef USE_EXTU
  memcpy(&ptr->rightExtU, extURight, sizeof(ExtU));
  memcpy(&ptr->leftExtU, extULeft, sizeof(ExtU));
#endif
#ifdef USE_EXTY
  memcpy(&ptr->rightExtY, extYRight, sizeof(ExtY));
  memcpy(&ptr->leftExtY, extYLeft, sizeof(ExtY));
#endif
#ifdef USE_POSITION_STATUS
  memcpy(&ptr->position, position, sizeof(PositionStatus));
#endif
}

#endif  // USE_MAIN_STATUS

#ifdef USE_MOTOR_CONTROL

uint16_T MotorControl_calcChecksum(MotorControl *ptr) {
  uint16_T checksum = (uint16_T)(ptr->start ^ ptr->steer ^ ptr->speed ^ ptr->modeType ^ ptr->standby);
#ifdef USE_SIDEBOARD_CONTROL
  checksum ^= (uint16_T)(ptr->ledCmd ^ ptr->ledMask);
#endif  
  return checksum;
}

#endif // USE_MOTOR_CONTROL


/* --------------------- POSITION_STATUS ------------------------------*/
#ifdef USE_POSITION_STATUS

uint16_T PositionStatus_calcChecksum(PositionStatus *ptr) {
  uint16_T checksum  = (uint16_T)(ptr->start ^ ptr->temp ^ ptr->roll ^ ptr->pitch ^ ptr->yaw ^ ptr->accX ^ ptr->accX ^ ptr->accZ ^ ptr->timestamp);
  return checksum;
}

#endif // USE_POSITION_STATUS

#ifdef USE_SIDEBOARD_CONTROL

uint16_T SideboardControl_calcChecksum(SideboardControl *ptr) {
  uint16_T checksum = (uint16_T)(ptr->start ^ ptr->cmd ^ ptr->mask);
  return checksum;
}

#endif // USE_SIDEBOARD_CONTROL

