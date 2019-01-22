/*
 * LoRaBoat_V0-2
 *  - Display debug on ST3775
 *  - Send LoRa message and listen for response
 *  - Support for following sensors
 *    + ICP10100          Atmospheric Pressure / Temperature
 *    + NEO-6M-0-001      GPS
 *    + Thermistor        Water Temperature
 *    + Thermistor        Air Temperature
 *    + KX224-I2C         Accelerometer
 *    + RPR-0521RS        Light Sensor
 *    + TSW-10            Turbidity Sensor
 * 
 * To add a new sensor:
 *  - Document the connections in the comments below
 *  - Add necessary libraries
 *  - Add necessary constants
 *  - Add sensor output to data struct
 *  - Add initX() and getX() functions as needed
 *  - Modify initGPIO if needed
 *  - Instantiate
 *  - Add init function to setup()
 *  - Add get functioin to getReadings()
 *  - Add TFT debug and LoRa print to sendLoRa()
 * 
 * Connections
 * 128 x 128 TFT Screen
 * ST3775                MKR WAN
 * -------------         -------
 * GND                    G         Black
 * VCC                    VCC       Red
 * RESET (RESET)          RESET     Yellow/Green
 * A0    (DC)             7         Blue/Purple                                                                                                                                                           
 * SDA   (MOSI)           8         orange
 * SCK   (SCK)            9         Brown/Yellow
 * CS    (CS)             6         White/Blue
 * LED+                   VCC       Red
 * LED-                   G         Black
 * 
 * GPS
 * NEO-6M-0-001          MKR WAN
 * ------------          -------
 * VCC                   VCC        Purple
 * TX                    RX         White    
 * RX                    TX         Grey
 * GND                   G          Black
 * 
 * H2O Thermistor        A6         Blue
 * Air Thermistor        A5         Purple
 * 
 * RPR-0521RS,
 * ICP10100, KX224-I2C   MKR WAN
 * -------------------   -------
 * VCC                   VCC        Red
 * GND                   GND        Black
 * SDA                   SDA        Green
 * SCL                   SCL        Yellow
 * 
 * TSW-10                MKR WAN
 * --------------------  -------
 * Pin 1, VCC            VCC
 * Pin 2, Photo TR       A4         Per datasheet, 4K7 R after take-off
 * Pin 3, LED            GND        Connected through 470R
 * 
 * Frank Milburn
 * December 2018
 */

//************************************* Libraries ***********************************
#include <MKRWAN.h>
#include "ArduinoLowPower.h"
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <icp101xx.h>
#include <Wire.h>
#include <KX224_I2C.h>
#include <RPR-0521RS.h>

//**************************** Definitions and Constants ****************************
#define DEBUG 0               // 0 = No debug
                              // 1 = Debug on TFT

// 1.8" tft screen 
#define TFT_CS   6
#define TFT_RST -1 
#define TFT_DC   7
// LoRa message sleep and listen intervals
const unsigned long    SLEEP_TIME  = 3000;      
const unsigned long    LISTEN_TIME = 200;
// GPS
static const uint32_t  GPS_BAUD = 9600;
static const double    HOME_LAT = 47.5239;
static const double    HOME_LON = -122.3864;
// Thermistors
#define TMP_SAMPLE     5
#define H2O_PIN        A6         
#define H2O_THERMNOM   10300
#define H2O_TEMPNOM    23.3
#define H2O_BCOEFF     3950
#define H2O_RESISTOR   9953    
#define AIR_PIN        A5       
#define AIR_THERMNOM   10300
#define AIR_TEMPNOM    23.3
#define AIR_BCOEFF     3950
#define AIR_RESISTOR   9953

// Turbidity Sensor
#define TURB_SAMPLE    5
#define TURBIDITY_PIN  A4

const char boatName[] = "MER_1";
const int numMsg = 15;
const int msgLen = 15;            // Description of outgoing values
const char msg[numMsg][msgLen]{
  "Msg Source  ",           //  0
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
  
//*********************************** Data Structs **********************************
struct data{
  double        latitude;
  double        longitude;
  double        distanceHome;
  unsigned long numMeasures;
  float         atmPress;
  float         light;
  float         boatTemp;
  float         h2oTemp;
  float         airTemp;
  float         acc[3];
  float         turbidity;
} readings;

//************************************* Functions ***********************************
void initGPIO();
void splashScreen();
void initLora();
void initPressure();
void initKx224();
void initRPR0521();
void initAnalogTemp();
void getReadings(struct data reading);
float getThermistor(int numSamples, int pin, float thermNom, float tempNom,
                    float Bcoeff, float resistor);
float getRPR0521();
void sendLoRa(struct data reading);
void listenLoRa();
float getTurbidity(int numSamples);

// *********************************** Instantiate **********************************
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);
TinyGPSPlus gps;
ICP101xx pressure;
KX224 kx224(KX224_DEVICE_ADDRESS_1E);
RPR0521RS rpr0521rs;

//--------------------------------------- Setup -------------------------------------
void setup(void) {

  initGPIO();
  Wire.begin();
  Serial1.begin(GPS_BAUD);
  splashScreen();
  initLoRa();
  initPressure();
  initKx224();
  initRPR0521();
  initAnalogTemp();
}

//---------------------------------------- Loop -------------------------------------
void loop() {

  getReadings(readings);
  sendLoRa(readings);
  listenLoRa();

  smartDelay(SLEEP_TIME);    // used to keep GPS active but minimize power
}

//------------------------------------- spashScreen ---------------------------------
void splashScreen() {
  
  // Initializer for 1.8" TFT (128 x 128) using ST7735S chip
  tft.initR(INITR_BLACKTAB); 
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

//-------------------------------------- initGPIO -----------------------------------
void initGPIO(){
  // set all pins to input and low
  int i;
  for (i = 0 ; i < NUM_DIGITAL_PINS ; i++ )
  {
    pinMode(i, OUTPUT ) ;
    pinMode(i, LOW);
  }
}

//-------------------------------------- initLoRa -----------------------------------
void initLoRa(){
  
  tft.setCursor(0, 0);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.println("Starting LoRa");
  if (!LoRa.begin(915E6)) {
    tft.println("LoRa start failed");
    while (1);
  }
  tft.println("LoRa started");
}

//------------------------------------ initPressure ---------------------------------
void initPressure(){
  
  pressure.begin();
}

//------------------------------------- getReadings ---------------------------------
void getReadings(struct data reading){

  readings.numMeasures++;
  readings.longitude = gps.location.lng();
  readings.latitude = gps.location.lat();
  readings.distanceHome =     
      TinyGPSPlus::distanceBetween(
      readings.latitude,
      readings.longitude,
      HOME_LAT, 
      HOME_LON) / 1000;
  pressure.measure(pressure.VERY_ACCURATE);
  readings.atmPress = pressure.getPressurePa();
  readings.boatTemp = pressure.getTemperatureC();
  readings.h2oTemp = getThermistor(TMP_SAMPLE, H2O_PIN, H2O_THERMNOM,
                                   H2O_TEMPNOM, H2O_BCOEFF, H2O_RESISTOR);
  readings.airTemp = getThermistor(TMP_SAMPLE, AIR_PIN, AIR_THERMNOM,
                                   AIR_TEMPNOM, AIR_BCOEFF, AIR_RESISTOR);
  kx224.get_val(readings.acc);
  readings.light = getRPR0521();
  readings.turbidity = getTurbidity(TURB_SAMPLE);
}

//-------------------------------------- sendLoRa -----------------------------------
void sendLoRa(struct data readings){

  if (DEBUG > 0){
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(0, 0);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_YELLOW);
    tft.print(" LoRa Buoy\n");
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_WHITE);
    tft.println("");
    tft.print(gps.date.year());     tft.print(":");
    tft.print(gps.date.month());    tft.print(":");
    tft.print(gps.date.day());      tft.print("  ");
    tft.print(gps.time.hour());     tft.print(":");
    tft.print(gps.time.minute());   tft.print(":");
    tft.println(gps.time.second()); tft.println("");
    tft.print(msg[0]); tft.println(boatName);
    tft.print(msg[1]); tft.println(readings.numMeasures);
    tft.print(msg[2]); tft.println(readings.longitude,4);
    tft.print(msg[3]); tft.println(readings.latitude,4);
    tft.print(msg[4]); tft.println(readings.distanceHome,3);
    tft.print(msg[5]); tft.println(readings.acc[0]);
    tft.print(msg[6]); tft.println(readings.acc[1]);
    tft.print(msg[7]); tft.println(readings.acc[2]);
    tft.print(msg[8]); tft.println(readings.atmPress,3);
    tft.print(msg[9]); tft.println(readings.light);
    tft.print(msg[10]);tft.println(readings.boatTemp);
    tft.print(msg[11]);tft.println(readings.h2oTemp);
    tft.print(msg[12]);tft.println(readings.airTemp);
    tft.print(msg[13]);tft.println(readings.turbidity);    
  }
  LoRa.beginPacket();
  LoRa.print(boatName);              LoRa.print(",");  // 0
  LoRa.print(readings.numMeasures);  LoRa.print(",");  // 1
  LoRa.print(readings.longitude);    LoRa.print(",");  // 2
  LoRa.print(readings.latitude);     LoRa.print(",");  // 3
  LoRa.print(readings.distanceHome); LoRa.print(",");  // 4
  LoRa.print(readings.acc[0]);       LoRa.print(",");  // 5
  LoRa.print(readings.acc[1]);       LoRa.print(",");  // 6
  LoRa.print(readings.acc[2]);       LoRa.print(",");  // 7
  LoRa.print(readings.atmPress);     LoRa.print(",");  // 8
  LoRa.print(readings.light);        LoRa.print(",");  // 9
  LoRa.print(readings.boatTemp);     LoRa.print(",");  // 10
  LoRa.print(readings.h2oTemp);      LoRa.print(",");  // 11
  LoRa.print(readings.airTemp);      LoRa.print(",");  // 12
  LoRa.print(readings.turbidity);                      // 13
  LoRa.endPacket();
}

//-------------------------------------listenLoRa -----------------------------------
void listenLoRa(){

  if (DEBUG > 0){
    tft.print("\nRSSI  [dB]: ");
  }
  unsigned long startListen = millis();
  while ((millis() - startListen) < LISTEN_TIME){
    // try to parse packet
    int packetSize = LoRa.parsePacket();
    if (packetSize > 0){
      while (LoRa.available()){
        char input = (char) LoRa.read();
        if (DEBUG > 0){
          tft.print(input);
        }
      }
    }
  }
  if (DEBUG > 0) {
    tft.println("");
  }
  LoRa.flush();
}

//------------------------------------- smartDelay ----------------------------------
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}

//------------------------------------ getThermistor --------------------------------
// Methodology from Adafruit: https://learn.adafruit.com/thermistor/using-a-thermistor
float getThermistor(int numSamples, int pin, float thermNom, float tempNom,
                    float Bcoeff, float resistor){
   
  uint32_t i;
  float average;
  uint32_t samples[numSamples];
 
  // take N samples in a row, with a slight delay
  for (i=0; i< numSamples; i++) {
   samples[i] = analogRead(pin);
   delay(10);
  }
 
  // average the samples
  average = 0;
  for (i=0; i< numSamples; i++) {
     average += samples[i];
  }
  average /= (float)numSamples;
 
  // convert the value to resistance
  average = 1023 / average - 1;
  average = resistor / average;
 
  float steinhart;
  steinhart = average / thermNom;              // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= Bcoeff;                         // 1/B * ln(R/Ro)
  steinhart += 1.0 / (tempNom + 273.15);       // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C

  return steinhart;
}

//------------------------------------- initKx224 -----------------------------------
void initKx224(){
  
  kx224.init();
}

//------------------------------------ initRPR0521 ----------------------------------
void initRPR0521(){
  
  rpr0521rs.init();
}

//------------------------------------- getRPR0521 ----------------------------------
float getRPR0521(){
  byte rc;
  unsigned short ps_val;
  float als_val;
  byte near_far;
  
  rc = rpr0521rs.get_psalsval(&ps_val, &als_val);
  return als_val;
}

//----------------------------------- initAnalogTemp --------------------------------
void initAnalogTemp(){

  pinMode(H2O_PIN, INPUT);
  pinMode(AIR_PIN, INPUT);
}

//------------------------------------ getTurbidity ---------------------------------
float getTurbidity(int numSamples){

  // NOTE:  This is a very rough and uncalibrated voltage / turbidity calc based
  // on running the sensor on 3V3 instead of 5V.  Turbidity not calculated.

  uint32_t i;
  float average;
  uint32_t samples[numSamples];
  
  // take numSamples in a row, with a slight delay and average
  for (i=0; i< numSamples; i++) {
   samples[i] = analogRead(TURBIDITY_PIN);
   delay(10);
  }
  // average the samples
  average = 0;
  for (i=0; i< numSamples; i++) {
     average += samples[i];
  }
  average /= (float)numSamples;

  // Convert to voltage
  float volts = 3.3 * average / (1023-1);

  // Return raw voltage corrected to 5V
  return volts * 2;
}
//..+....1....!....+....2....+....3....+....4....+....5....+....6....+....7....+....8
