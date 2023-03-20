#include <AvioticsESP32.h>
#include "usartData.h"
#include "StreamWebHandler.h"

StreamWebHandler::StreamWebHandler() {
  serial = NULL;
  idx = 0;
  bufStartFrame = 0;
  p = NULL;
  incomingByte = (byte)0;
  incomingBytePrev = (byte)0;
  failures = 0L;
  successes = 0L;
  oldChecksum = newChecksum = 0;

  printf("Size of MainStatus: %d\n", sizeof(MainStatus));
};

StreamWebHandler::~StreamWebHandler() {
};

void StreamWebHandler::init(HardwareSerial *serial) {
  this->serial = serial;
};

esp_err_t StreamWebHandler::request(httpd_req_t *req, const char* buf, size_t bufSize, char *resp, size_t respSize) {
  esp_err_t ret = ESP_OK;
  size_t partBufSize = 64;
  uint8_t partBuf[partBufSize];
  if (serial == NULL)
    return ret;
  
  Serial.printf("Core %d: requestSequence %d\n", xPortGetCoreID(), buf);
  char printBuf[512];
  int printLength = 0;
  printLength = printMainStatusHeader(printBuf, printLength);
  printf(printBuf);
  printf("Size: %d\n", printLength);
  sendChunk(req, printLength, (uint8_t*)printBuf, partBufSize, partBuf, -1);

  int count = 0;
  while (true) {
    // Check for new data availability in the Serial buffer
    if (serial->available()) {
      incomingByte = serial->read(); // Read the incoming byte
      bufStartFrame	= ((uint16_t)(incomingByte) << 8) | incomingBytePrev;  // Construct the start frame
    }
    else {
      delay(10);
      continue;
    }
    // Copy received data
    if (bufStartFrame == MAIN_STATUS_START_FRAME) {	                    // Initialize if new data is detected
        p    = (byte *)&feedback;
        *p++ = incomingBytePrev;
        *p++ = incomingByte;
        idx  = 2;	
    }
    else if (idx >= 2 && idx < sizeof(MainStatus)) {  // Save the new received data
        *p++    = incomingByte; 
        idx++;
    }	
    
    // Check if we reached the end of the package
    if (idx == sizeof(MainStatus)) {
        uint16_t checksum;
        checksum   = MainStatus_calcChecksum(&feedback);

        // Check validity of the new data
        if (feedback.start == MAIN_STATUS_START_FRAME && checksum == feedback.checksum) {
            successes++;
            // Copy the new data
            // size_t dataLength = sizeof(MainStatus);
            // memcpy(&oldFeedback, &feedback, dataLength);
            oldFeedback = feedback;
            printLength = 0;
            printLength = printMainStatus(printBuf, printLength, &oldFeedback);
            // printf(printBuf);
            count++;
            // ret = sendChunk(req, dataLength, (uint8_t*)&OldFeedback, partBufSize, partBuf, count);
            if (printLength > 0)
              ret = sendChunk(req, printLength, (uint8_t*)printBuf, partBufSize, partBuf, -1);

        } else {
          failures++;
          Serial.printf("Non-valid data skipped failure rate=%12.2f '%'\n", 100.0*failures/(failures+successes));
        }
        idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
    }
    // Update previous states
    incomingBytePrev = incomingByte;

  }
  return ret;
};

void StreamWebHandler::serialReceiveLoop() {
  if (serial == NULL)
    return;
  
  Serial.printf("Core %d\n", xPortGetCoreID());
  int count = 0;
  while (true) {
    // Check for new data availability in the Serial buffer
    if (serial->available()) {
      incomingByte = serial->read(); // Read the incoming byte
      bufStartFrame	= ((uint16_t)(incomingByte) << 8) | incomingBytePrev;  // Construct the start frame
    }
    else {
      delay(10);
      continue;
    }
    // Copy received data
    if (bufStartFrame == MAIN_STATUS_START_FRAME) {	                    // Initialize if new data is detected
        p    = (byte *)&feedback;
        *p++ = incomingBytePrev;
        *p++ = incomingByte;
        idx  = 2;	
    }
    else if (idx >= 2 && idx < sizeof(MainStatus)) {  // Save the new received data
        *p++    = incomingByte; 
        idx++;
    }	
    
    // Check if we reached the end of the package
    if (idx == sizeof(MainStatus)) {
        uint16_t checksum;
        checksum   = MainStatus_calcChecksum(&feedback);

        // Check validity of the new data
        if (feedback.start == MAIN_STATUS_START_FRAME && checksum == feedback.checksum) {
            successes++;
            // Copy the new data
            // size_t dataLength = sizeof(MainStatus);
            // memcpy(&oldFeedback, &feedback, dataLength);
            oldFeedback = feedback;

            count++;
        } else {
          failures++;
          Serial.printf("Non-valid data skipped failure rate=%12.2f '%'\n", 100.0*failures/(failures+successes));
        }
        idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
    }
    // Update previous states
    incomingBytePrev = incomingByte;

  }
};


/**
 * This method is used for stream callbacks i.e. for cases where a multipart request is done
 * and these are the chunks
 */
esp_err_t StreamWebHandler::sendChunk(void *reqPtr, size_t dataLength, uint8_t *data, size_t partBufSize, uint8_t *partBuf, int count) {
  int nmax = 100;
  httpd_req_t *req = (httpd_req_t*)reqPtr;
  esp_err_t res = ESP_OK;
  
  if (count > 0 && ((count % nmax) == 1)) {
    size_t hlen = snprintf((char *)partBuf, partBufSize, com::aviotics::web::WifiWebServer::STREAM_PART, dataLength*nmax);
    res = httpd_resp_send_chunk(req, (const char *)partBuf, hlen);
  }
  if(res == ESP_OK) {
    res = httpd_resp_send_chunk(req, (const char *)data, dataLength);
  }
  else
    printf("sendChunk: send data failed\n");
  if(count > 0 && ((count % nmax) == 0) && res == ESP_OK)
    res = httpd_resp_send_chunk(req, com::aviotics::web::WifiWebServer::STREAM_BOUNDARY, strlen(com::aviotics::web::WifiWebServer::STREAM_BOUNDARY));

  return res;
}

int StreamWebHandler::printExtUHeader(char *buf, int offs) {
  return offs + sprintf(&buf[offs], "b_motEna\tz_ctrlModReq\tr_inpTgt\tb_hallA\tb_hallB\tb_hallC\ti_phaAB\ti_phaBC\ti_DCLink\ta_mechAngle");
}

int StreamWebHandler::printExtU(char *buf, int offs, ExtU *extU) {
  return offs + sprintf(&buf[offs], "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d", extU->b_motEna, extU->z_ctrlModReq, extU->r_inpTgt, extU->b_hallA, extU->b_hallB, extU->b_hallC, extU->i_phaAB, extU->i_phaBC, extU->i_DCLink, extU->a_mechAngle);
}

int StreamWebHandler::printExtYHeader(char *buf, int offs) {
  return offs + sprintf(&buf[offs], "DC_phaA\tDC_phaB\tDC_phaC\tz_errCode\tn_mot\ta_elecAngle\tiq\tid\ttimestamp");
}

int StreamWebHandler::printExtY(char *buf, int offs, ExtY *extY) {
  return offs + sprintf(&buf[offs], "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d", extY->DC_phaA, extY->DC_phaB, extY->DC_phaC, extY->z_errCode, extY->n_mot, extY->a_elecAngle, extY->iq, extY->id, extY->timestamp);
}

int StreamWebHandler::printPositionHeader(char *buf, int offs) {
  return offs + sprintf(&buf[offs], "temp\troll\tpitch\tyaw\taccX\taccY\taccZ\ttimestamp");
}

int StreamWebHandler::printPosition(char *buf, int offs, PositionStatus *pos) {
  return offs + sprintf(&buf[offs], "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d", pos->temp, pos->roll, pos->pitch, pos->yaw, pos->accX, pos->accY, pos->accZ, pos->timestamp);
}

int StreamWebHandler::printMainStatusHeader(char *buf, int offs) {
  size_t nmax = sizeof(MainStatus);
  memset(buf, 0, nmax);
  uint16_t version = 1;
  uint32_t processorTime = millis();
  uint64_t realTime = 0L;
  sprintf(buf, "%d\tHoverserial\t%d\t%d\t%ld END\n", nmax, version, processorTime, realTime);
  offs = nmax;
  /*
  offs = printExtUHeader(buf, offs);
  offs += sprintf(&buf[offs], "\t");
  offs = printExtUHeader(buf, offs);
  offs += sprintf(&buf[offs], "\t");
  offs = printExtYHeader(buf, offs);
  offs += sprintf(&buf[offs], "\t");
  offs = printExtYHeader(buf, offs);
  offs += sprintf(&buf[offs], "\t");
  offs = printPositionHeader(buf, offs);
  offs += sprintf(&buf[offs], "\t");
  offs += sprintf(&buf[offs], "cmd1\tcmd2\tspeedR_meas\tspeedL_meas\tbatVoltage\tboardTemp\tcmdLed");
  offs += sprintf(&buf[offs], "\n");
  */
  return offs;
}

int StreamWebHandler::printMainStatus(char *buf, int offs, MainStatus *status) {
  newChecksum = status->checksum;
  if (newChecksum == oldChecksum)
    return 0;
  oldChecksum = newChecksum;
  size_t nmax = sizeof(MainStatus);
  memset(buf, 0, nmax);
  memccpy(buf, status, 1, nmax);
  offs = nmax;
  /*
  offs = printExtU(buf, offs, &status->rightExtU);
  offs += sprintf(&buf[offs], "\t");
  offs = printExtU(buf, offs, &status->leftExtU);
  offs += sprintf(&buf[offs], "\t");
  offs = printExtY(buf, offs, &status->rightExtY);
  offs += sprintf(&buf[offs], "\t");
  offs = printExtY(buf, offs, &status->leftExtY);
  offs += sprintf(&buf[offs], "\t");
  offs = printPosition(buf, offs, &status->position);
  offs += sprintf(&buf[offs], "\t");
  offs += sprintf(&buf[offs], "%d\t%d\t%d\t%d\t%d\t%d\t%d", status->cmd1, status->cmd2, status->speedR_meas, status->speedL_meas, status->batVoltage, status->boardTemp, status->cmdLed);
  offs += sprintf(&buf[offs], "\n");
  */
  return offs;
}

const MainStatus* StreamWebHandler::getFeedback() {
  return &feedback;
}
