// *******************************************************************
//  Arduino Nano 5V example code
//  for   https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC
//
//  Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
//
// *******************************************************************
// INFO:
// • This sketch uses the the Serial Software interface to communicate and send commands to the hoverboard
// • The built-in (HW) Serial interface is used for debugging and visualization. In case the debugging is not needed,
//   it is recommended to use the built-in Serial interface for full speed perfomace.
// • The data packaging includes a Start Frame, checksum, and re-syncronization capability for reliable communication
// 
// The code starts with zero speed and moves towards +
//
// CONFIGURATION on the hoverboard side in config.h:
// • Option 1: Serial on Right Sensor cable (short wired cable) - recommended, since the USART3 pins are 5V tolerant.
//   #define CONTROL_SERIAL_USART3
//   #define FEEDBACK_SERIAL_USART3
//   // #define DEBUG_SERIAL_USART3
// • Option 2: Serial on Left Sensor cable (long wired cable) - use only with 3.3V devices! The USART2 pins are not 5V tolerant!
//   #define CONTROL_SERIAL_USART2
//   #define FEEDBACK_SERIAL_USART2
//   // #define DEBUG_SERIAL_USART2
// *******************************************************************

// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD     	// [-] Start frame definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval
#define SPEED_MAX_TEST      300         // [-] Maximum speed for testing
#define SPEED_STEP          10          // [-] Speed step
// #define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

#define LED_BUILTIN 2
#define RXD2 16
#define TXD2 17

// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  uint8_t b_motEna;                  /* '<Root>/b_motEna' */
  uint8_t z_ctrlModReq;                /* '<Root>/z_ctrlModReq' */
  int16_t r_inpTgt;                    /* '<Root>/r_inpTgt' */
  uint8_t b_hallA;                     /* '<Root>/b_hallA ' */
  uint8_t b_hallB;                     /* '<Root>/b_hallB' */
  uint8_t b_hallC;                     /* '<Root>/b_hallC' */
  int16_t i_phaAB;                     /* '<Root>/i_phaAB' */
  int16_t i_phaBC;                     /* '<Root>/i_phaBC' */
  int16_t i_DCLink;                    /* '<Root>/i_DCLink' */
  int16_t a_mechAngle;                 /* '<Root>/a_mechAngle' */
} ExtU;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  int16_t DC_phaA;                     /* '<Root>/DC_phaA' */
  int16_t DC_phaB;                     /* '<Root>/DC_phaB' */
  int16_t DC_phaC;                     /* '<Root>/DC_phaC' */
  uint8_t z_errCode;                   /* '<Root>/z_errCode' */
  int16_t n_mot;                       /* '<Root>/n_mot' */
  int16_t a_elecAngle;                 /* '<Root>/a_elecAngle' */
  int16_t iq;                          /* '<Root>/iq' */
  int16_t id;                          /* '<Root>/id' */
  uint32_t timestamp;
} ExtY;

uint16_t ExtU_calcChecksum(ExtU *thisA) {
  uint16_t checksum = thisA->b_motEna ^ thisA->z_ctrlModReq ^ thisA->r_inpTgt ^ thisA->b_hallA
    ^ thisA->b_hallB ^ thisA->b_hallC ^ thisA->i_phaAB ^ thisA->i_phaBC ^ thisA->i_DCLink ^ thisA->a_mechAngle;
    return checksum;
}

uint16_t ExtY_calcChecksum(ExtY *thisA) {
  uint16_t checksum = thisA->DC_phaA ^ thisA->DC_phaB ^ thisA->DC_phaC ^ thisA->z_errCode
    ^ thisA->n_mot ^ thisA->a_elecAngle ^ thisA->iq ^ thisA->id ^ thisA->timestamp;
    return checksum;
}


typedef struct{
  uint16_t  start;
  int16_t   cmd1;
  int16_t   cmd2;
  int16_t   speedR_meas;
  int16_t   speedL_meas;
  int16_t   batVoltage;
  int16_t   boardTemp;
  uint16_t  cmdLed;

  ExtU      rightExtU;
  ExtU      leftExtU;
  ExtY      rightExtY;
  ExtY      leftExtY;

  uint16_t  checksum;
} SerialFeedback;

SerialFeedback OldFeedback;
SerialFeedback Feedback;

// ########################## SETUP ##########################
void setup() 
{
  Serial.begin(SERIAL_BAUD);
  while (!Serial);
  Serial.println("Hoverboard Serial ESP32 v1.0");

  Serial2.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, RXD2, TXD2);
  while (!Serial2);

  pinMode(LED_BUILTIN, OUTPUT);
}

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  Serial2.write((uint8_t *) &Command, sizeof(Command)); 
}

// ########################## RECEIVE ##########################
void Receive() {
    // Check for new data availability in the Serial buffer
    if (Serial2.available()) {
        incomingByte 	  = Serial2.read();                                   // Read the incoming byte
        bufStartFrame	= ((uint16_t)(incomingByte) << 8) | incomingBytePrev;       // Construct the start frame
    }
    else {
        return;
    }

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_RX
        Serial.print(incomingByte);
        return;
    #endif

    // Copy received data
    if (bufStartFrame == START_FRAME) {	                    // Initialize if new data is detected
        p       = (byte *)&Feedback;
        *p++    = incomingBytePrev;
        *p++    = incomingByte;
        idx     = 2;	
    } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
        *p++    = incomingByte; 
        idx++;
    }	
    
    // Check if we reached the end of the package
    if (idx == sizeof(SerialFeedback)) {
        uint16_t checksum;
        checksum   = (uint16_t)(Feedback.start ^ Feedback.cmd1 ^ Feedback.cmd2 ^ Feedback.speedR_meas ^ Feedback.speedL_meas 
                                           ^ Feedback.batVoltage ^ Feedback.boardTemp ^ Feedback.cmdLed);
        checksum ^= ExtU_calcChecksum(&(Feedback.rightExtU)) ^ ExtU_calcChecksum(&(Feedback.leftExtU)) ^ ExtY_calcChecksum(&(Feedback.rightExtY)) ^ ExtY_calcChecksum(&(Feedback.rightExtY));

        // Check validity of the new data
        if (Feedback.start == START_FRAME && checksum == Feedback.checksum) {
            // Copy the new data
            memcpy(&OldFeedback, &Feedback, sizeof(SerialFeedback));

            // Print data to built-in Serial
            Serial.println("");
            Serial.print("1(cmd1): ");   Serial.print(OldFeedback.cmd1);
            Serial.print(" 2(cmd2): ");  Serial.print(OldFeedback.cmd2);
            Serial.print(" 3(speedR): ");  Serial.print(OldFeedback.speedR_meas);
            Serial.print(" 4(speedL): ");  Serial.print(OldFeedback.speedL_meas);
            Serial.print(" 5(batt.Volt): ");  Serial.print(OldFeedback.batVoltage);
            Serial.print(" 6(temp): ");  Serial.print(OldFeedback.boardTemp);
            Serial.print(" 7(LED): ");  Serial.println(OldFeedback.cmdLed);
            Serial.println("");
        } else {
          Serial.println("Non-valid data skipped");
        }
        idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
    }

    // Update previous states
    incomingBytePrev = incomingByte;
}

// ########################## LOOP ##########################

unsigned long iTimeSend = 0;
int iTest = 0;
int iStep = SPEED_STEP;

void loop(void) { 
  unsigned long timeNow = millis();

  // Check for new received data
  Receive();

  // Send commands
  if (iTimeSend > timeNow) return;
  iTimeSend = timeNow + TIME_SEND;
  Send(0, iTest);

  // Calculate test command signal
  iTest += iStep;

  // invert step if reaching limit
  if (iTest >= SPEED_MAX_TEST || iTest <= -SPEED_MAX_TEST)
    iStep = -iStep;

  // Blink the LED
  digitalWrite(LED_BUILTIN, (timeNow%2000)<1000);
}

// ########################## END ##########################
