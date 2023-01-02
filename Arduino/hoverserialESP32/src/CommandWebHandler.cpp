#include <AvioticsESP32.h>
#include "usartData.h"
#include "CommandWebHandler.h"

CommandWebHandler::CommandWebHandler() {
  motorControl = NULL;
};

CommandWebHandler::~CommandWebHandler() {
};

void CommandWebHandler::init(MotorControl *motorControl) {
  this->motorControl = motorControl;
};

esp_err_t CommandWebHandler::requestSequence(const char* buf, size_t bufSize) {
  String str = buf;
  str.replace("%20", "   ");
  buf = str.c_str();
   Serial.printf("requestSequence %s\n", buf);
   return ESP_OK;
}

void CommandWebHandler::process() {
}

void CommandWebHandler::reset() {
}

String CommandWebHandler::getData() {
  return "";
}

esp_err_t CommandWebHandler::request(httpd_req_t *req, const char* buf, size_t bufSize, char *resp, size_t respSize) {
  // Serial.printf("Request on core %d\n", xPortGetCoreID());
  // Serial.println(xPortGetCoreID());
  Serial.printf("WebHandler::request: %s\n", buf);
  String json = "";
  char variable[1024] = {0,};
  motorControl->start = MOTOR_CONTROL_START_FRAME;
  if (httpd_query_key_value(buf, "ble", variable, sizeof(variable)) == ESP_OK) {
    json = getData();
    Serial.println(json);
  }
  
  variable[0] = 0;
  if (httpd_query_key_value(buf, "leds", variable, sizeof(variable)) == ESP_OK) {
    motorControl->ledCmd = atoi(variable);
    motorControl->ledMask = 255;
    Serial.printf("motorControl: %d\n", motorControl->ledCmd);
    variable[0] = 0;
  }
  if (httpd_query_key_value(buf, "rightSpeed", variable, sizeof(variable)) == ESP_OK) {
    motorControl->speed = atoi(variable);
    variable[0] = 0;
  }
  if (httpd_query_key_value(buf, "leftSpeed", variable, sizeof(variable)) == ESP_OK) {
    motorControl->steer = atoi(variable);
    variable[0] = 0;
  }
  if (httpd_query_key_value(buf, "modeType", variable, sizeof(variable)) == ESP_OK) {
    motorControl->modeType = atoi(variable);
    variable[0] = 0;
  }

  if (json.length() < 1)
    json = "{\"ret\":\"OK\"}";
  strcpy(resp, json.c_str());
  return ESP_OK;
};

