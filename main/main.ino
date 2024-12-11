/**************************************************************************

  CAN Gauge Capstone Project
  Sean Minami
  CSCI490

 **************************************************************************/

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#include <mcp2515_can.h>

#define DEBUG 1

#define X_OFFSET 240
#define Y_OFFSET 320
#define ZERO_PERCENT_ANGLE 42.27
#define HUNDRED_PERCENT_ANGLE -41.01

// SPI CS and reset pins for TFT display
#define TFT_CS        10
#define TFT_RST        7
#define TFT_DC         8

//For CAN-BUS shield
#define CAN_CS_PIN    9

//For button
#define BUTTON_INT_PIN 3

//Arduino and engine CAN IDs
#define OBD2_GATEWAY_ID   0x7DF
#define OBD2_ENGINE_ID    0x7E8

//parameters to monitor
#define COOLANT_PID       0x05
#define ENGINE_RPM_PID    0x0C
#define IGN_TIMING_PID    0x0E
#define INTAKE_TEMP_PID   0x0F
#define DTC_SCAN          0xFF

//scanning interval
#define INTERVAL 50

//limits for parameters
#define MAX_RPM   8000

// For 1.14", 1.3", 1.54", 1.69", and 2.0" TFT with ST7789:
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

mcp2515_can CAN(CAN_CS_PIN);

unsigned long time = millis();
unsigned long previousMillis = 0;
const unsigned long oneSecond = 1000;

double p = 3.1415926;
double percent = 0;
double data = 0;

int8_t currentMode = 0;

int16_t lastx = 0;
int16_t lasty = 0;

unsigned char sensorData[8];

volatile bool changeDisplay = false;

unsigned char modes[5] = {ENGINE_RPM_PID, COOLANT_PID, IGN_TIMING_PID, INTAKE_TEMP_PID, DTC_SCAN};

char code[6];

void setup(void) {
  #if DEBUG
  SERIAL_PORT_MONITOR.begin(115200);
  while(!Serial){};
  SERIAL_PORT_MONITOR.println(F("Start Gauge Demo"));
  SERIAL_PORT_MONITOR.flush();
  #endif

  tft.init(240, 320);           // Init ST7789 320x240

  tft.fillScreen(ST77XX_BLACK); //draw gauge and needle at 0%
  drawGauge();
  drawNeedle();

  pinMode(BUTTON_INT_PIN, INPUT_PULLUP);  //enable internal 20kOhm pullup
  attachInterrupt(digitalPinToInterrupt(BUTTON_INT_PIN), handleChangeInterrupt, FALLING); //attach int handler function to button pin, triggered on falling edge

  #if DEBUG
  SERIAL_PORT_MONITOR.println(F("Display Initialized"));
  SERIAL_PORT_MONITOR.flush();
  #endif


  while(CAN_OK != CAN.begin(CAN_500KBPS)){
    #if DEBUG
    SERIAL_PORT_MONITOR.println(F("CAN initialization failed, trying again"));
    SERIAL_PORT_MONITOR.flush();
    #endif
    delay(1000);
  }

  #if DEBUG
  SERIAL_PORT_MONITOR.println(F("CAN initialized"));
  SERIAL_PORT_MONITOR.flush();
  #endif
  initRecvFilter(); //only recieve messages with ECU's sender id
 
}

void loop() {
  if(CAN_MSGAVAIL == CAN.checkReceive()) {
    previousMillis = millis();
    #if DEBUG
    SERIAL_PORT_MONITOR.println(F("Pending message recieved"));
    SERIAL_PORT_MONITOR.println(double(millis() - time)/oneSecond);
    SERIAL_PORT_MONITOR.flush();
    #endif

    for(int i = 0; i<8; i++){
      sensorData[i] = 0;
    }

    readCanBuf();

    #if DEBUG
    SERIAL_PORT_MONITOR.println(F("buffer filled"));
    SERIAL_PORT_MONITOR.flush();
    #endif

    switch(modes[currentMode]){
      case ENGINE_RPM_PID:
        calcEngineRPM();
        break;
      case COOLANT_PID:
        calcCoolantTemp();
        break;
      case IGN_TIMING_PID:
        calcIgnTiming();
        break;
      case INTAKE_TEMP_PID:
        calcIntakeTemp();
        break;
      default:
        data = 0;
        percent = 0;
        break;
    }
    
  } else if(millis() - previousMillis >= INTERVAL) {
    previousMillis = millis();
    #if DEBUG
    SERIAL_PORT_MONITOR.println(F("No incoming messages, sending request"));
    SERIAL_PORT_MONITOR.println(double(millis() - time)/oneSecond);
    SERIAL_PORT_MONITOR.flush();
    #endif
    if(!changeDisplay){
      sendDataRequest(modes[currentMode]);

    } else {
      changeDisplay = false;
      if(currentMode < 4){ //if on live data, increment mode.  If on DTC screen, return to first selection
        currentMode = currentMode + 1;
      } else {
        currentMode = 0;
      }

      if(modes[currentMode] == DTC_SCAN){
        int bytesToGo = 0;
        int codesToGo = 0;
        int currentCodeRaw = 0;
        unsigned long requestSent = 0;

        setupDTCScreen();
        sendDTCRequest();
        requestSent = millis();
        for(int i = 0; i<8; i++){
          sensorData[i] = 0;
        }
        while(millis() - requestSent < 100){};
        tft.setCursor(0,20);
        tft.setTextSize(2);
        tft.setTextColor(0xFFFF);
        if(readCanBuf()){
          if((sensorData[0] & 0xF0) == 0){  //if recieved single frame response (no subsequent data)
            bytesToGo = int(sensorData[0] & 0x0F);  //lower 4 of first byte gives number of bytes response
            codesToGo = int(sensorData[2]);   //third byte is number of codes returned
            
            for(int i = 0; i < codesToGo*2; i+=2){
              currentCodeRaw = 256 * int(sensorData[i+3]) + int(sensorData[i+4]);
              getErrorCode(currentCodeRaw);
              tft.println(code);
            }
            
            

          } else if((sensorData[0] & 0xF0) == 1){ //else if multi frame response
            bytesToGo = 256*int(sensorData[0] & 0x0F) + int(sensorData[1]); //lower 4 of first byte and second byte gives number of bytes returned
            codesToGo = int(sensorData[3]); //4th byte gives number of DTC recieved
            unsigned char prevByte = 0x00;
            
            for(int i = 0; i < 4; i+=2){  //for first frame
              currentCodeRaw = 256 * int(sensorData[i+4]) + int(sensorData[i+5]);
              getErrorCode(currentCodeRaw);
              bytesToGo -= 2;
              codesToGo -= 1;
              tft.println(code);
            }
            bool oddFrame = true;
            sendDTCFlowControlFrame();
            requestSent = millis();
            for(int i = 0; i<8; i++){
              sensorData[i] = 0;
            }
            while(millis() - requestSent < 100){};
            readCanBuf();
            for(int i = 0; i < codesToGo; i++){
              if(i == 3 && oddFrame){
                i -= 4;
                codesToGo -= 3;
                oddFrame = false;
                prevByte = sensorData[7];
                readCanBuf();
              } else if(i == 3 && !oddFrame){
                i -= 4;
                codesToGo -= 4;
                currentCodeRaw = 256 * int(sensorData[2*i+2]) + int(sensorData[2*i+3]);
                getErrorCode(currentCodeRaw);
                tft.println(code);
              }else if (i == 0 && !oddFrame){
                currentCodeRaw = 256 * int(prevByte) + int(sensorData[1]);
                prevByte = 0x00;
                getErrorCode(currentCodeRaw);
                tft.println(code);

              } else if(oddFrame){
                currentCodeRaw = 256 * int(sensorData[2*i+1]) + int(sensorData[2*i+2]);
                getErrorCode(currentCodeRaw);
                tft.println(code);

              } else if(!oddFrame){
                currentCodeRaw = 256 * int(sensorData[2*i+2]) + int(sensorData[2*i+3]);
                getErrorCode(currentCodeRaw);
                tft.println(code);
              }
            }
          }
        }
        while(!changeDisplay){};
      } else {
        drawGauge();
        drawNeedle();
      }
    }  
  }
}

void getErrorCode(int rawCode){
  sprintf(code, "ERR");
  switch(rawCode & 0xF0000000){ //first 2 bits are DTC prefix, rest is DTC code
    case 0:
      code[0] = 'P';
      sprintf(code + 1, "%04lX", (rawCode & 0x0FFFFFFF));
      break;
    case 1:
      code[0] = 'C';
      sprintf(code + 1, "%04lX", (rawCode & 0x0FFFFFFF));
      break;
    case 2:
      code[0] = 'B';
      sprintf(code + 1, "%04lX", (rawCode & 0x0FFFFFFF));
      break;
    case 3:
      code[0] = 'U';
      sprintf(code + 1, "%04lX", (rawCode & 0x0FFFFFFF));
      break;
    default:
      break;
  }
}

void initRecvFilter(){
  //Set same mask for both recv buffers
  //11-bit mask, so 7FC means top 9 bits of sender ID will be compared with filters
  CAN.init_Mask(0,0, 0x7FC);
  CAN.init_Mask(1,0, 0x7FC);
  //define filters for those buffers 0-1 apply to buffer 0 2-5 for buffer 1
  //only allow messages with engine's sender id

  //buffer 0 filters
  CAN.init_Filt( 0, 0, OBD2_ENGINE_ID);
  CAN.init_Filt( 1, 0, OBD2_ENGINE_ID);

  //buffer 1 filters
  CAN.init_Filt( 2, 0, OBD2_ENGINE_ID);
  CAN.init_Filt( 3, 0, OBD2_ENGINE_ID);
  CAN.init_Filt( 4, 0, OBD2_ENGINE_ID);
  CAN.init_Filt( 5, 0, OBD2_ENGINE_ID);
}

void sendDataRequest(unsigned char CAN_PID){ 
  //message structure: {[2 bytes of data], [mode 1], [sensor to query], [5 bytes padding]}
  //mode 1: read live data
  unsigned char req[8] = {0x02, 0x01, CAN_PID, 0x00, 0x00, 0x00, 0x00, 0x00};
  #if DEBUG
  SERIAL_PORT_MONITOR.print(F("sendDataRequest(): Requesting data from: 0x"));
  SERIAL_PORT_MONITOR.println(CAN_PID, HEX);
  SERIAL_PORT_MONITOR.println(double(millis() - time)/oneSecond);
  SERIAL_PORT_MONITOR.flush();
  #endif
  CAN.sendMsgBuf(OBD2_GATEWAY_ID, 0, 8, req);
}

void sendDTCFlowControlFrame(){
  unsigned char req[8] = {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  #if DEBUG
  SERIAL_PORT_MONITOR.println(F("Requesting Consecutive Frame"));
  SERIAL_PORT_MONITOR.flush();
  #endif
  CAN.sendMsgBuf(OBD2_GATEWAY_ID, 0, 8, req);
}

void sendDTCRequest(){  //send DTC request packet
  //message structure: {[1 byte of data], [mode 3], [6 bytes padding]}
  //mode 3: show stored DTCs
  unsigned char req[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  #if DEBUG
  SERIAL_PORT_MONITOR.println(F("Requesting DTCs"));
  SERIAL_PORT_MONITOR.flush();
  #endif
  CAN.sendMsgBuf(OBD2_GATEWAY_ID, 0, 8, req);
}

void calcEngineRPM(){
  data = ((256*int(sensorData[3]) + int(sensorData[4])) * 0.25);
  percent = data * 100 / double(MAX_RPM);
  #if DEBUG
  SERIAL_PORT_MONITOR.println(F("Raw upper byte:"));
  SERIAL_PORT_MONITOR.println(sensorData[3]);
  SERIAL_PORT_MONITOR.println(F("Raw lower byte:"));
  SERIAL_PORT_MONITOR.println(sensorData[4]);
  SERIAL_PORT_MONITOR.flush();
  SERIAL_PORT_MONITOR.print(F("RPM: "));
  SERIAL_PORT_MONITOR.println(data);
  SERIAL_PORT_MONITOR.println(double(millis() - time)/oneSecond);
  SERIAL_PORT_MONITOR.flush();
  #endif
  updateNeedle(percent);
  updateDigital(data, 0);
  updateDigitalUnits(ENGINE_RPM_PID);
}

void calcCoolantTemp(){
  data = (int(sensorData[3]) - 40);
  #if DEBUG
  SERIAL_PORT_MONITOR.println(F("Raw upper byte:"));
  SERIAL_PORT_MONITOR.println(sensorData[3]);
  SERIAL_PORT_MONITOR.println(F("Calculated upper byte:"));
  SERIAL_PORT_MONITOR.println((int(sensorData[3])-40));
  SERIAL_PORT_MONITOR.print(F("COOLANT: "));
  SERIAL_PORT_MONITOR.println(data);
  SERIAL_PORT_MONITOR.flush();
  #endif
  if(data < 50){  // if below 50c
    percent = 0;
  } else if(data > 130){  //if above 130c, set needle to max. pray for the user
    percent = 100;
  } else {
    percent = 100 * (data - 50) / 130;
  }

  updateNeedle(percent);
  updateDigital(data, 0);
  updateDigitalUnits(COOLANT_PID);
}

void calcIntakeTemp(){
  data = int(sensorData[3]) - 40;
#if DEBUG
  SERIAL_PORT_MONITOR.println(F("Raw upper byte:"));
  SERIAL_PORT_MONITOR.println(sensorData[3]);
  SERIAL_PORT_MONITOR.println(F("Calculated upper byte:"));
  SERIAL_PORT_MONITOR.println(int(sensorData[3]) - 40);
  SERIAL_PORT_MONITOR.print(F("INTAKE: "));
  SERIAL_PORT_MONITOR.println(data);
  SERIAL_PORT_MONITOR.flush();
#endif
  if(data > 90){    //if above 90c, set needle to max
    percent = 100;
  } else if(data < 0){
    percent = 0;
  } else {
    percent = 100 * data / 90.0;
  }
  updateNeedle(percent);
  updateDigital(data, 0);
  updateDigitalUnits(INTAKE_TEMP_PID);
}

void calcIgnTiming(){
  data = (int(sensorData[3]) / 2.0) - 64;
  percent = 100 *int(sensorData[3]) / 255.0;  // 0 deg timing advance BTDC should read in middle of gauge
  #if DEBUG
  SERIAL_PORT_MONITOR.println(F("Raw upper byte:"));
  SERIAL_PORT_MONITOR.println(sensorData[3]);
  SERIAL_PORT_MONITOR.println(F("Calculated upper byte:"));
  SERIAL_PORT_MONITOR.println((int(sensorData[3])-40));
  SERIAL_PORT_MONITOR.print(F("TIMING: "));
  SERIAL_PORT_MONITOR.println(data);
  SERIAL_PORT_MONITOR.flush();
  #endif
  updateNeedle(percent);
  updateDigital(data, 1);
  updateDigitalUnits(IGN_TIMING_PID);
}

int readCanBuf(){  //get pending message
  unsigned char msglen = 0;
  CAN.readMsgBuf(&msglen, sensorData);
  int canid = CAN.getCanId();

  #if DEBUG
    SERIAL_PORT_MONITOR.println(F("readCanBuf(): start CAN.readMsgBuf()"));
    SERIAL_PORT_MONITOR.print(F("readCanBuf(): From CAN ID:"));
    SERIAL_PORT_MONITOR.println(canid, HEX);
    SERIAL_PORT_MONITOR.println(double(millis() - time)/oneSecond);
    SERIAL_PORT_MONITOR.flush();
  #endif

  #if DEBUG
    SERIAL_PORT_MONITOR.println(F("readCanBuf(): buffer filled"));
    SERIAL_PORT_MONITOR.print(F("readCanBuf(): msglen = "));
    SERIAL_PORT_MONITOR.println(int(msglen));
    SERIAL_PORT_MONITOR.println(double(millis() - time)/oneSecond);
    SERIAL_PORT_MONITOR.flush();
  #endif

  #if DEBUG
  SERIAL_PORT_MONITOR.println(F(""));
  SERIAL_PORT_MONITOR.print(F("Get Data from: 0x:"));
  SERIAL_PORT_MONITOR.println(canid, HEX);
  SERIAL_PORT_MONITOR.flush();
  for(int i = 0; i < int(msglen); i++){
    SERIAL_PORT_MONITOR.print(F("0x"));
    SERIAL_PORT_MONITOR.print(sensorData[i], HEX);
    SERIAL_PORT_MONITOR.print(F("\t"));
  }
  SERIAL_PORT_MONITOR.flush();
  SERIAL_PORT_MONITOR.println();
  SERIAL_PORT_MONITOR.println(double(millis() - time)/oneSecond);
  SERIAL_PORT_MONITOR.println();
  SERIAL_PORT_MONITOR.flush();
  #endif

  return int(msglen);
}

//draw semicircle for gauge
void drawGauge(){
  tft.fillScreen(ST77XX_BLACK);
  tft.drawCircle(120, 200, 119, 0xFFFF);
  tft.fillRect(0, 280, 240, 40, 0x0000);
}
//draw needle at 0
void drawNeedle(){
  lastx = 32;
  lasty = 280;
  tft.drawLine(32, 280, 120, 200, 0xF000);
}

void setupDTCScreen(){
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0,0);
  tft.setTextSize(2);
  tft.setTextColor(0xFFFF);
  tft.print(F("Stored Error Codes:"));
}

void updateDigital(double data, int decimals){
  tft.fillRect(0, 0, 240, 80, 0x0000);
  tft.setCursor(80, 40);
  tft.setTextSize(5);
  tft.setTextColor(0xFFFF);
  tft.print(data, decimals);
}

void updateDigitalUnits(unsigned char parameter){
  char units[12];
  switch(parameter){
    case ENGINE_RPM_PID:
      strncpy(units, "RPM", 12);
      break;
    case COOLANT_PID:
      strncpy(units, "Water Temp", 12);
      break;
    case IGN_TIMING_PID:
      strncpy(units, "Deg BTDC", 12);
      break;
    case INTAKE_TEMP_PID:
      strncpy(units, "Intake Temp", 12);
      break;
    default:
      strncpy(units, "", 12);
      break;
  }
  tft.fillRect(0, 280, 240, 40, 0x0000);
  int centerOffset = strlen(units)/2;
  tft.setCursor(120-(centerOffset*20), 280);
  tft.setTextSize(3);
  tft.setTextColor(0xFFFF);
  tft.print(units);
}

void updateNeedle(double percent){
  double angle = percent * -2.6328 + 42;
  angle = angle * p / 180;
  double xval = 118 * cos(angle) + 120;
  double yval = 118 * sin(angle) + 200;
  int16_t xcoord = round(xval);
  int16_t ycoord = round(yval);
  tft.drawLine(lastx, lasty, 120, 200, 0x0000);
  tft.drawLine(X_OFFSET - xcoord, ycoord, 120, 200, 0xF000);
  lastx = X_OFFSET-xcoord;
  lasty = ycoord;
}

void handleChangeInterrupt(){
  static unsigned long last_interrupt = 0;
  unsigned long interrupt_now = millis();
  if(interrupt_now - last_interrupt > 500){
    changeDisplay = true;
  }
  last_interrupt = millis();
}

