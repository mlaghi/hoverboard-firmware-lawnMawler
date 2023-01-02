#ifndef _COMMAND_WEB_HANDLER_H
#define _COMMAND_WEB_HANDLER_H

#include <AvioticsESP32.h>
#include "usartData.h"

class CommandWebHandler : public com::aviotics::web::WebServerInterface {
  private:
    MotorControl *motorControl;

  public:
    CommandWebHandler();
    ~CommandWebHandler();
    void init(MotorControl *motorControl);
    esp_err_t requestSequence(const char* buf, size_t bufSize);
    void process();
    void reset();
    String getData();
    esp_err_t request(httpd_req_t *req, const char* buf, size_t bufSize, char *resp, size_t respSize);
};

#endif
