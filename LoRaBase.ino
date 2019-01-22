/*
 * LoRaBas_V0-7
 * Responds with RSSI after receiving a message
 * Incorporates 1.8" SPI/TFT 128x160
 * Parses incoming messages and sends to TFT
 * Labels each message
 * Provides string output for connection to Raspberry Pi and adafruit.io
 */

//**************************** Definitions and Constants ****************************
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_GFX.h>         // Core graphics library
#include <Adafruit_ST7735.h>      // Hardware-specific library for ST7735

#define SERIAL_BAUD  115200

#define DEBUG 0                   // = 0    TFT output with labels
                                  // = 1    TFT and string to serial
                                  // = 2    TFT and serial output with labels
// 1.8" tft screen 
#define TFT_CS   6
#define TFT_RST -1 
#define TFT_DC   7

#define MSG_BUFFER_SIZE 100       // Buffer size for incoming message

const int numMsg = 15;
const int msgLen = 15;            // chars in descriptions of parsed incoming messags
const char msg[numMsg][msgLen]{
  "Source      ",           //  0
  "Msg   [No]  ",           //  1
  "Long  [deg] ",           //  2
  "Lat   [deg] ",           //  3
  "Dist  [km]  ",           //  4
  "Acc X [g]   ",           //  5
  "Acc Y [g]   ",           //  6
  "Acc Z [g]   ",           //  7
  "Atm P [Pa]  ",           //  8
  "Light [lx]  ",           //  9
  "Boat T[C]   ",           // 10
  "H2O T [C]   ",           // 11
  "Air T [C]   ",           // 12
  "Turb  [V]   ",           // 13
  "RSSI  [dB]  "};          // 14
  
// *********************************** Instantiate **********************************
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

//--------------------------------------- Setup -------------------------------------
void setup() {
  if(DEBUG > 1){
    Serial.begin(SERIAL_BAUD);
    delay(1000);
  }
  
  splashScreen();

  if (!LoRa.begin(915E6)) {
    tft.println("LoRa MER Base start failed");
    if (DEBUG > 1) Serial.println("LoRa MER Base start failed");
    while (1);
  }
}

//---------------------------------------- Loop -------------------------------------
void loop() {
  char message[MSG_BUFFER_SIZE];
  int messageLen = 0;
  message[0] = '\0';
  
  // check for packets
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet, read and store in char array message
    while (LoRa.available()) {
      char incoming = (char) LoRa.read();
      if (messageLen < MSG_BUFFER_SIZE) {
        message[messageLen] = incoming;
        messageLen++;
        message[messageLen] = '\0';
      }     
    }
    int rssi = LoRa.packetRssi();
    
    tftMsgOutput(message, rssi);

    if (DEBUG == 1){
      Serial.println(message);           // pyserial seems to want a new line
    }
   
    if (DEBUG > 1){
      serialOutput(message, rssi);
    }
        
    // Now respond to sender with RSSI
    LoRa.beginPacket();
    LoRa.print(rssi);
    LoRa.endPacket();
  }
}

//------------------------------------- spashScreen ---------------------------------
void splashScreen() {
  
  // Initializer for 1.8" TFT (128 x 128) using ST7735S chip
  tft.initR(INITR_BLACKTAB); 

  tft.setRotation(2);
  tft.fillScreen(ST77XX_BLACK);    
  tft.setTextWrap(false);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 10);
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(3);
  tft.println("element");
  tft.println("   14   ");
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(2);
  tft.println("  Build a  ");
  tft.println("  Smarter");
  tft.println("   World ");
  tft.setTextColor(ST77XX_WHITE);
  tft.println("  Arduino");
  tft.println("  MKR WAN");
  tft.println("   1300  ");
  
  delay(2000);
  tft.fillScreen(ST77XX_BLACK);
}

//------------------------------------- tftMsgOutput ----------------------------------
void tftMsgOutput(char inMessage[], int rssi){

  char message[strlen(inMessage)+1];
  strcpy(message, inMessage);

  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 0);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_YELLOW);
  tft.print(" LoRa MER\n");
  tft.setTextSize(1);
  tft.print("    Base Station\n");
  tft.setTextColor(ST77XX_WHITE);
  tft.println("");


  int iMsg = 0;
  char* msgPtr;
  msgPtr = strtok(message, ",");           // parse comma separated values
  while (msgPtr != '\0'){
    tft.print(msg[iMsg]);
    iMsg++;
    tft.println(msgPtr);
    msgPtr = strtok('\0',",");
  }
  tft.println("");   
  tft.print(msg[numMsg-1]);
  tft.println(rssi);
}


//------------------------------------- serialOutput ----------------------------------
void serialOutput(char inMessage[], int rssi){

  char message[strlen(inMessage)+1];
  strcpy (message, inMessage);

  int iMsg = 0;
  char* msgPtr;
  msgPtr = strtok(message, ",");           // parse comma separated values
  while (msgPtr != '\0'){
    Serial.print(msg[iMsg]);
    iMsg++;
    Serial.println(msgPtr);
    msgPtr = strtok('\0',",");
  }
  Serial.println("");   
  Serial.print(msg[numMsg-1]);
  Serial.println(rssi);
  Serial.println("\n");
}
