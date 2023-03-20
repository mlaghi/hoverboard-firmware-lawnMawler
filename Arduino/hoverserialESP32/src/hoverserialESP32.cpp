#include "CommandWebHandler.h"
#include "StreamWebHandler.h"
#include "HCSR04Sensor.h"
#include "Locator.h"

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
bool blockWrites = false;

// #define INPUT_ECHO = GPIO_NUM_23
// #define OUTPUT_SOUND = GPIO_NUM_22

HCSR04Sensor usSensor(GPIO_NUM_23);


void checkPerformance() {
	com::aviotics::motion::Locator locator;
	PosDir posDir;
	PosDir_init(&posDir, 40.0, 1.0, 0.0);
	locator.fillRectangleObstacles(0.0, 0.0, 50.0, 40.0, 0.05);
	uint32_t t0 = millis();
	FLOAT val = 0.0;
	for (int i=0; i < 100; i++)
		val = locator.calcSignal(posDir);
	uint32_t t1 = millis();
	Serial.printf("Distance: %f\t%d\n", val, t1-t0);
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
  webServer.addWifiAccess("NewLaghi2023!", "Laghi2023!");

  webServer.init(&cmdWebHandler, &streamWebHandler, 80, "mic00", "frifra20", 35);
  Serial.println(webServer.getIP());

  pinMode(LED_BUILTIN, OUTPUT);
  usSensor.setTemperature(20);

  // checkPerformance();

}

uint16_t sentChecksum = 0;
int count = 0;


// ########################## SEND ##########################
void send() {
  // Create command
  command.start    = (uint16_t)MOTOR_CONTROL_START_FRAME;
  if (blockWrites && command.ledCmd == 255)
    return;
  blockWrites = false;
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
  // Serial.printf("Send over UART %d\n", count++);
  if (command.ledCmd == 255)
    blockWrites = true;
}

// ########################## LOOP ##########################

unsigned long iTimeSend = 0;
int iTest = 0;
int iStep = SPEED_STEP;

uint32_t lastOutput = 0L;

void loopMeasurement(void) {
  usSensor.startMeasurement();
  int32_t delta = usSensor.getResult();
  uint32_t now = millis();
  if (now - lastOutput > 30000)
    usSensor.setDebug(true);
  if (delta > -1) {
    lastOutput = now;
    Serial.printf("time: %d [us]\tdistance=%7.3f [m]\n", delta, usSensor.getMeters(delta));
  }
  delay(1);
}

void loop(void) { 
  unsigned long timeNow = millis();

  // Check for new received data

  // Send commands
  if (iTimeSend > timeNow) return;
  iTimeSend = timeNow + TIME_SEND;
  // send(0, iTest);
  send();

  // Calculate test command signal
  iTest += iStep;

  // invert step if reaching limit
  if (iTest >= SPEED_MAX_TEST || iTest <= -SPEED_MAX_TEST)
    iStep = -iStep;

  // Blink the LED
  digitalWrite(LED_BUILTIN, (timeNow%2000)<1000);
}

// ########################## END ##########################
