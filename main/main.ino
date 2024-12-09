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
//#define CAN_INT_PIN   2

//For button
#define BUTTON_INT_PIN 3

//Arduino and engine CAN IDs
#define OBD2_GATEWAY_ID   0x7DF
#define OBD2_ENGINE_ID    0x7E8

//parameters to monitor
#define COOLANT_PID       0x05
#define MANIFOLD_KPA_PID  0x0B
#define ENGINE_RPM_PID    0x0C
#define IGN_TIMING_PID    0x0E
#define INTAKE_TEMP_PID   0x0F
#define DTC_SCAN          0xFF


//limits for parameters
#define MAX_RPM   8000

// For 1.14", 1.3", 1.54", 1.69", and 2.0" TFT with ST7789:
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

mcp2515_can CAN(CAN_CS_PIN);

double p = 3.1415926;
double percent = 0;
double data = 0;

int8_t currentMode = 0;

int16_t lastx = 0;
int16_t lasty = 0;

unsigned char* sensorData;

volatile bool changeDisplay = false;

unsigned char modes[6] = {ENGINE_RPM_PID, COOLANT_PID, MANIFOLD_KPA_PID, IGN_TIMING_PID, INTAKE_TEMP_PID, DTC_SCAN};

void setup(void) {
  #if DEBUG
  Serial.begin(115200);
  Serial.println(F("Start Gauge Demo"));
  #endif

  tft.init(240, 320);           // Init ST7789 320x240

  tft.fillScreen(ST77XX_BLACK); //draw gauge and needle at 0%
  drawGauge();
  drawNeedle();

  pinMode(BUTTON_INT_PIN, INPUT_PULLUP);  //enable internal 20kOhm pullup
  attachInterrupt(digitalPinToInterrupt(BUTTON_INT_PIN), handleChangeInterrupt, FALLING); //attach int handler function to button pin, triggered on falling edge

  //DEMO-----
  // updateNeedle(873.0*100/MAX_RPM);
  // updateDigital(873);
  // tft.setCursor(90, 290);
  // tft.setTextSize(3);
  // tft.setTextColor(0xFFFF);
  // tft.print("RPM");
  //DEMO-----

  #if DEBUG
  Serial.println(F("Display Initialized"));
  #endif

  while(CAN_OK != CAN.begin(CAN_500KBPS)){
    #if DEBUG
    Serial.println("CAN initialization failed, trying again");
    #endif
    delay(1000);
  }

  #if DEBUG
  Serial.println("CAN initialized");
  #endif
  initRecvFilter(); //only recieve messages with ECU's sender id
 
}

void loop() {
  //percent += increment;
  //Serial.println(percent);

  
  if(CAN.checkReceive() != CAN_MSGAVAIL){
    #if DEBUG
    Serial.println("No incoming messages, sending request");
    #endif
    if(!changeDisplay){ //TODO: MODE SWITCHING
      sendDataRequest(modes[currentMode]);
    } else {
      changeDisplay = false;
      if(currentMode < 4){
        currentMode = currentMode + 1;
      } else {
        currentMode = 0;
      }
    }
    // } else {
    //   changeDisplay = false;
    //   if(currentMode < 5){ //if on DTC scan mode, set mode to 0 else increment mode
    //     currentMode = currentMode + 1;
    //   } else {
    //     currentMode = 0;
    //   }

    //   if(modes[currentMode] == DTC_SCAN){
    //     //TODO: display DTCs
    //     int bytesToGo = 0;
    //     int codesToGo = 0;
    //     int currentCodeRaw = 0;
    //     char code[6];

    //     setupDTCScreen();
    //     sendDTCRequest();
    //     tft.setCursor(0,40);
    //     tft.setTextSize(1);
    //     tft.setTextColor(0xFFFF);
    //     delay(15);
    //     readCanBuf();
    //     if((sensorData[0] & 0xF0) == 0){  //if recieved single frame response (no subsequent data)
    //       bytesToGo = int(sensorData[0] & 0x0F);  //lower 4 of first byte gives number of bytes response
    //       codesToGo = int(sensorData[2]);   //third byte is number of codes returned
          
    //       for(int i = 0; i < codesToGo*2; i+=2){
    //         currentCodeRaw = 256 * int(sensorData[i+3]) + int(sensorData[i+4]);
    //         switch(currentCodeRaw & 0xF0000000){
    //           case 0:
    //             code[0] = 'P';
    //             sprintf(code + 1, "%04li", (currentCodeRaw & 0x0FFFFFFF));
    //             break;
    //           case 1:
    //             code[0] = 'C';
    //             sprintf(code + 1, "%04li", (currentCodeRaw & 0x0FFFFFFF));
    //             break;
    //           case 2:
    //             code[0] = 'B';
    //             sprintf(code + 1, "%04li", (currentCodeRaw & 0x0FFFFFFF));
    //             break;
    //           case 3:
    //             code[0] = 'U';
    //             sprintf(code + 1, "%04li", (currentCodeRaw & 0x0FFFFFFF));
    //             break;
    //           default:
    //             break;
    //         }
    //         tft.println(code);
    //       }
          
          

    //     } else if((sensorData[0] & 0xF0) == 1){ //else if multi frame response
    //       bytesToGo = 256*int(sensorData[0] & 0x0F) + int(sensorData[1]); //lower 4 of first byte and second byte gives number of bytes returned
    //       codesToGo = int(sensorData[3]); //4th byte gives number of DTC recieved
    //       unsigned char prevByte = 0x00;
    //       bool incompleteFrame = true;

    //       for(int i = 0; i < 4; i+=2){  //for first frame
    //         currentCodeRaw = 256 * int(sensorData[i+4]) + int(sensorData[i+5]);
    //         switch(currentCodeRaw & 0xF0000000){  //first 2 bits are DTC prefix, all else error code
    //           case 0:
    //             code[0] = 'P';
    //             sprintf(code + 1, "%04li", (currentCodeRaw & 0x0FFFFFFF));
    //             break;
    //           case 1:
    //             code[0] = 'C';
    //             sprintf(code + 1, "%04li", (currentCodeRaw & 0x0FFFFFFF));
    //             break;
    //           case 2:
    //             code[0] = 'B';
    //             sprintf(code + 1, "%04li", (currentCodeRaw & 0x0FFFFFFF));
    //             break;
    //           case 3:
    //             code[0] = 'U';
    //             sprintf(code + 1, "%04li", (currentCodeRaw & 0x0FFFFFFF));
    //             break;
    //           default:
    //             break;
    //         }
    //         bytesToGo -= 2;
    //         codesToGo -= 1;
    //         tft.println(code);
    //       }
    //       sendDTCFlowControlFrame();
    //       delay(15);
    //       readCanBuf();
    //       for(int i = 0; i < codesToGo; i++){
    //         if(i == 3 && incompleteFrame){
    //           i -= 4;
    //           codesToGo -= 3;
    //           incompleteFrame = false;
    //           prevByte = sensorData[7];
    //           readCanBuf();
    //           delay(15);
    //         } else if (i == 0 && !incompleteFrame){
    //           currentCodeRaw = 256 * int(prevByte) + int(sensorData[1]);
    //           prevByte = 0x00;
    //           switch(currentCodeRaw & 0xF0000000){  //first 2 bits are DTC prefix, all else error code
    //             case 0:
    //               code[0] = 'P';
    //               sprintf(code + 1, "%04li", (currentCodeRaw & 0x0FFFFFFF));
    //               break;
    //             case 1:
    //               code[0] = 'C';
    //               sprintf(code + 1, "%04li", (currentCodeRaw & 0x0FFFFFFF));
    //               break;
    //             case 2:
    //               code[0] = 'B';
    //               sprintf(code + 1, "%04li", (currentCodeRaw & 0x0FFFFFFF));
    //               break;
    //             case 3:
    //               code[0] = 'U';
    //               sprintf(code + 1, "%04li", (currentCodeRaw & 0x0FFFFFFF));
    //               break;
    //             default:
    //               break;
    //           }
    //           tft.println(code);

    //         } else if(incompleteFrame){
    //           currentCodeRaw = 256 * int(sensorData[2*i+1]) + int(sensorData[2*i+2]);
    //           switch(currentCodeRaw & 0xF0000000){  //first 2 bits are DTC prefix, all else error code
    //             case 0:
    //               code[0] = 'P';
    //               sprintf(code + 1, "%04li", (currentCodeRaw & 0x0FFFFFFF));
    //               break;
    //             case 1:
    //               code[0] = 'C';
    //               sprintf(code + 1, "%04li", (currentCodeRaw & 0x0FFFFFFF));
    //               break;
    //             case 2:
    //               code[0] = 'B';
    //               sprintf(code + 1, "%04li", (currentCodeRaw & 0x0FFFFFFF));
    //               break;
    //             case 3:
    //               code[0] = 'U';
    //               sprintf(code + 1, "%04li", (currentCodeRaw & 0x0FFFFFFF));
    //               break;
    //             default:
    //               break;
    //           }
    //           tft.println(code);
    //           if(i == 3){
    //             i -= 4;
    //             codesToGo -= 3;
    //             incompleteFrame = false;
    //             readCanBuf();
    //             delay(15);
    //           }
    //         } else if(!incompleteFrame){
    //           currentCodeRaw = 256 * int(sensorData[2*i+2]) + int(sensorData[2*i+3]);
    //           switch(currentCodeRaw & 0xF0000000){  //first 2 bits are DTC prefix, all else error code
    //             case 0:
    //               code[0] = 'P';
    //               sprintf(code + 1, "%04li", (currentCodeRaw & 0x0FFFFFFF));
    //               break;
    //             case 1:
    //               code[0] = 'C';
    //               sprintf(code + 1, "%04li", (currentCodeRaw & 0x0FFFFFFF));
    //               break;
    //             case 2:
    //               code[0] = 'B';
    //               sprintf(code + 1, "%04li", (currentCodeRaw & 0x0FFFFFFF));
    //               break;
    //             case 3:
    //               code[0] = 'U';
    //               sprintf(code + 1, "%04li", (currentCodeRaw & 0x0FFFFFFF));
    //               break;
    //             default:
    //               break;
    //           }

    //           tft.println(code);
            
    //         }
    //       }
    //     }
    //     while(!changeDisplay){
    //       delay(200);
    //     }
    //   }
    // }

    //DELETE IF NECESSARY:
    //delay(20);
  } else {
    #if DEBUG
    Serial.println("Pending message recieved");
    #endif
    int CANid = CAN.getCanId();
    readCanBuf();

    switch(modes[currentMode]){
      case ENGINE_RPM_PID:  //DONE
        data = ((256*int(sensorData[3]) + int(sensorData[4])) * 0.25);
        percent = data * 100 / double(MAX_RPM);
        #if DEBUG
        Serial.print("CAN ID:");
        Serial.println(CANid);
        Serial.println("Raw upper byte:");
        Serial.println(sensorData[3]);
        Serial.println("Raw lower byte:");
        Serial.println(sensorData[4]);
        Serial.println("Calculated upper byte:");
        Serial.println((256*int(sensorData[3])));
        Serial.print("RPM: ");
        Serial.println(data);
        #endif
        updateNeedle(percent);
        updateDigital(data, 0);
        break;
      case COOLANT_PID: //DONE
        data = (int(sensorData[3]) - 40);
        #if DEBUG
        Serial.println("Raw upper byte:");
        Serial.println(sensorData[3]);
        Serial.println("Calculated upper byte:");
        Serial.println((int(sensorData[3])-40));
        Serial.print("COOLANT: ");
        Serial.println(data);
        #endif
        if(data < 50){  // if below 50c
          percent = 0;
        } else if(data > 130){  //if above 130c, set needle to max. pray for the user
          percent = 100;
        } else {
          percent = (data - 50) / 130;
        }

        updateNeedle(percent);
        updateDigital(data, 0);
        //updateDigitalUnits()?
        break;
      case MANIFOLD_KPA_PID:  //DONE
        data = int(sensorData[3]) - 101.325;  //convert from absolute to relative kPa
        #if DEBUG
        Serial.println("Raw upper byte:");
        Serial.println(sensorData[3]);
        Serial.println("Calculated upper byte:");
        Serial.println(int(sensorData[3]) - 101.325);
        Serial.print("PSI: ");
        Serial.println(data);
        #endif
        if(data < 0){   //if vacuum
          percent = 25 * ((data + 101.325)/101.325);  //atmospheric pressure at 25% on gauge
          data = -data * 0.2961;  //convert relative kPa to inHg vacuum
        } else {  //if boost
          percent = 25 + 75 * (data / (255-101.325));
          data = data * 0.145;  //convert relative kPa to PSI pressure
        }

        updateNeedle(percent);
        updateDigital(data, 1);
        //updateDigitalUnits()?
        break;
      case IGN_TIMING_PID:  //TODO: FIX SCALE
        data = int(sensorData[3]) / 2.0 - 64; //FIX SCALE
        percent = (data + 64) / 127.5;  // 0 deg timing advance BTDC should read in middle of gauge
        #if DEBUG
        Serial.println("Raw upper byte:");
        Serial.println(sensorData[3]);
        Serial.println("Calculated upper byte:");
        Serial.println((int(sensorData[3])-40));
        Serial.print("COOLANT: ");
        Serial.println(data);
        #endif
        updateNeedle(percent);
        updateDigital(data, 1);
        break;
      case INTAKE_TEMP_PID:   //DONE
        data = int(sensorData[3]) - 40;
        if(data > 90){    //if above 90c, set needle to max
          percent = 100;
        } else if(data < 0){
          percent = 0;
        } else {
          percent = data / 90;
        }
        updateNeedle(percent);
        updateDigital(data, 0);
        break;
      default:
        data = 0;
        percent = 0;
        break;
    }
    
    //sendDataRequest(modes[currentMode]);
    delay(100);
  }
  // delay(10);
  // if(percent >= 100 || percent <= 0){
  //   increment = increment * -1;
  // }
}

void initRecvFilter(){  //DONE
  //Set same mask for both recv buffers
  //11-bit mask, so 7FF means all bits of sender ID will be compared with filters
  CAN.init_Mask(0,0, 0x7FF);
  CAN.init_Mask(1,0, 0x7FF);
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

void sendDataRequest(unsigned char CAN_PID){  //DONE
  //message structure: {[2 bytes of data], [mode 1], [sensor to query], [5 bytes padding]}
  //mode 1: read live data
  unsigned char req[8] = {0x02, 0x01, CAN_PID, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC};
  #if DEBUG
  Serial.print("Requesting data from: 0x");
  Serial.println(CAN_PID, HEX);
  #endif
  CAN.sendMsgBuf(OBD2_GATEWAY_ID, 0, 8, req);
  delay(100);
}

void sendDTCFlowControlFrame(){
  unsigned char req[8] = {0x30, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC};
  #if DEBUG
  Serial.print("Requesting Consecutive Frame");
  #endif
  CAN.sendMsgBuf(OBD2_GATEWAY_ID, 0, 8, req);
  delay(100);
}

void sendDTCRequest(){  //send DTC request packet
  //message structure: {[1 byte of data], [mode 3], [6 bytes padding]}
  //mode 3: show stored DTCs
  unsigned char req[8] = {0x01, 0x03, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC};
  #if DEBUG
  Serial.print("Requesting DTCs");
  #endif
  CAN.sendMsgBuf(OBD2_GATEWAY_ID, 0, 8, req);
  //delay(20);
}

void readCanBuf(){  //get pending message
  unsigned char msglen = 0;

  CAN.readMsgBuf(&msglen, sensorData);
  delay(100);

  #if DEBUG
  Serial.println("------------------");
  Serial.print("Get Data from: 0x:");
  Serial.println(CAN.getCanId(), HEX);
  for(int i = 0; i < msglen; i++){
    Serial.print("0x");
    Serial.print(sensorData[i], HEX);
    Serial.print("\t");
  }
  Serial.println();
  #endif

  //return buf;
}

//draw semicircle for gauge
void drawGauge(){   //DONE
  tft.fillScreen(ST77XX_BLACK);
  tft.drawCircle(120, 200, 119, 0xFFFF);
  tft.fillRect(0, 280, 240, 40, 0x0000);
}

void drawNeedle(){  //DONE
  lastx = 32;
  lasty = 280;
  tft.drawLine(32, 280, 120, 200, 0xF000);
}

void setupDTCScreen(){  //tweak
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(40,20);
  tft.setTextSize(2);
  tft.setTextColor(0xFFFF);
  tft.print("Stored Trouble Codes:");
}

void updateDigital(double data, int decimals){  //tweak
  tft.fillRect(X_OFFSET-0, 0, -240, 80, 0x0000);
  tft.setCursor(80, 40);
  tft.setTextSize(5);
  tft.setTextColor(0xFFFF);
  tft.print(data, decimals);
}

void updateNeedle(double percent){  //DONE
  //drawGauge();
  double angle = percent * -2.6328 + 42;
  angle = angle * p / 180;
  double xval = 118 * cos(angle) + 120;
  double yval = 118 * sin(angle) + 200;
  //Serial.println(xval);
  int16_t xcoord = round(xval);
  int16_t ycoord = round(yval);
  //Serial.println(xcoord);
  tft.drawLine(lastx, lasty, 120, 200, 0x0000);
  tft.drawLine(X_OFFSET - xcoord, ycoord, 120, 200, 0xF000);
  lastx = X_OFFSET-xcoord;
  lasty = ycoord;
}

void handleChangeInterrupt(){   //DONE
  changeDisplay = true;
}

// void testdrawtext(char *text, uint16_t color) {
//   tft.setCursor(0, 0);
//   tft.setTextColor(color);
//   tft.setTextWrap(true);
//   tft.print(text);
// }

