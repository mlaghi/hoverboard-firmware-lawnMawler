#include "CommandWebHandler.h"
#include "StreamWebHandler.h"

#define SERIAL_BAUD         115200
#define TIME_SEND           100         // [ms] Sending time interval
#define SPEED_MAX_TEST      1500        // [-] Maximum speed for testing
#define SPEED_STEP          40          // [-] Speed step

#define LED_BUILTIN 2
#define RXD2 16
#define TXD2 17

MotorControl command;
com::aviotics::web::WifiWebServer webServer;
char webBuf[512];

CommandWebHandler cmdWebHandler;
StreamWebHandler streamWebHandler;

// End of WEB Declaration --------------------------------------------

int speed = 0;
int steer = 0;
int ledCmd = 0;
int ledMask = 0;

void readSerialVal() { // currently unused
  if (Serial.available() > 0) {
    String txt = Serial.readString();
    txt.trim();
    int currentSize;
    sscanf(txt.c_str(), "%d %d %d %d", &speed, &steer, &ledCmd, &ledMask);
    Serial.printf("%d %d %d %d\n", speed, steer, ledCmd, ledMask);
  }
}

// ########################## SETUP ##########################
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Hoverboard Serial ESP32 v1.0");

  Serial2.begin(SERIAL_BAUD, SERIAL_8N1, RXD2, TXD2);
  while (!Serial2);

  cmdWebHandler.init(&command);
  streamWebHandler.init(&Serial2);  
  webServer.addWifiAccess("mic08", "frifra20");
  webServer.addWifiAccess("laghi2022", "N0T4u2now");

  webServer.init(&cmdWebHandler, &streamWebHandler, 80, "mic00", "frifra20", 35);
  Serial.println(webServer.getIP());

  pinMode(LED_BUILTIN, OUTPUT);
}

uint16_t sentChecksum = 0;

// ########################## SEND ##########################
void send(int16_t uSteer, int16_t uSpeed, uint8_t ledCmd, uint8_t ledMask) {
  // Create command
  command.start    = (uint16_t)MOTOR_CONTROL_START_FRAME;
  /*
  command.steer    = (int16_t)uSteer;
  command.speed    = (int16_t)uSpeed;
  command.ledCmd   = (uint8_t)ledCmd;
  command.ledMask  = (uint8_t)ledMask;
  */
  command.checksum = MotorControl_calcChecksum(&command);
  if (command.checksum != sentChecksum) {
    Serial.printf("Command %d %d %d %d\t\t%d\n", command.speed, command.steer, command.ledCmd, command.ledMask, command.checksum);
    sentChecksum = command.checksum;
  }
  // Write to Serial
  Serial2.write((uint8_t *) &command, sizeof(command));
}

// ########################## LOOP ##########################

unsigned long iTimeSend = 0;
int iTest = 0;
int iStep = SPEED_STEP;

void loop(void) { 
  unsigned long timeNow = millis();

  // Check for new received data

  // Send commands
  if (iTimeSend > timeNow) return;
  iTimeSend = timeNow + TIME_SEND;
  // send(0, iTest);
  send(steer, speed, ledCmd, ledMask);

  // Calculate test command signal
  iTest += iStep;

  // invert step if reaching limit
  if (iTest >= SPEED_MAX_TEST || iTest <= -SPEED_MAX_TEST)
    iStep = -iStep;

  // Blink the LED
  digitalWrite(LED_BUILTIN, (timeNow%2000)<1000);
}

// ########################## END ##########################
