#ifndef _STREAM_WEB_HANDLER_H
#define _STREAM_WEB_HANDLER_H

#include <AvioticsESP32.h>
#include "usartData.h"

class StreamWebHandler : public com::aviotics::web::WebServerInterface {
  private:
    HardwareSerial *serial;

    uint8_t idx;             // Index for new data pointer
    uint16_t bufStartFrame;  // Buffer Start Frame
    byte *p;                 // Pointer declaration for the new received data
    byte incomingByte;
    byte incomingBytePrev;

    MainStatus OldFeedback;
    MainStatus Feedback;

    int printExtUHeader(char *buf, int offs);
    int printExtU(char *buf, int offs, ExtU *extU);
    int printExtYHeader(char *buf, int offs);
    int printExtY(char *buf, int offs, ExtY *extY);
    int printPosition(char *buf, int offs, PositionStatus *pos);
    int printPositionHeader(char *buf, int offs);
    int printMainStatusHeader(char *buf, int offs);
    int printMainStatus(char *buf, int offs, MainStatus *status);
    esp_err_t sendChunk(void *reqPtr, size_t dataLength, uint8_t *data, size_t partBufSize, uint8_t *partBuf, int count);

  public:
    StreamWebHandler();
    ~StreamWebHandler();
    void init(HardwareSerial *serial);
    esp_err_t request(httpd_req_t *req, const char* buf, size_t bufSize, char *resp, size_t respSize);
};

#endif
