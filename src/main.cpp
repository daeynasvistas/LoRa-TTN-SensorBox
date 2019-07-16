/* ************************************************************** 
 * Arduino sketch 
 * BME280 sensor and display them on the screen
 * Modified 9-7-2019 by Daey To add Co2 sensor end functional deepsleep
 * *************************************************************/
#include <SPI.h>
#include <Wire.h>

//SENSORES
#include "Adafruit_CCS811.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <sound_meter.h>

/* ************************************************************** 
 * Sensor setup
 * *************************************************************/
// BME280 + Co2
#define SEALEVELPRESSURE_HPA (1032.00)
Adafruit_BME280 bme; // I2C
Adafruit_CCS811 ccs; // I2C

// SOUND
int soundPin = 37; 
int sensorValue = 0; 
float soundValue = 0.0; 

// LUX
int luxPin = 38; // select the input pin for the potentiometer
float luxrawRange = 4096;
float luxlogRange = 4.6;
int LuxSensorValue = 0;

// sensor variables
float tempC = 0.0;
float pressure = 0.0;
float humidity = 0.0;
float altitudeMeter = 0.0;

float eCO2 = 0.0;
float TVOC  = 0.0;


#define LEDPIN 2
#define ANALOG_PIN       35
#define ANALOG_MAX_VALUE 4095

#define ACT_METHOD_ABP// define the activation method ABP or OTAA

//#define DEBUG
#define CFG_eu868

/* **************************************************************
* keys for device (removed for forum posting)
* *************************************************************/
static const uint8_t PROGMEM NWKSKEY[16] = { 0xE5, 0xD9, 0xC1, 0x31, 0x7F, 0xEC, 0x82, 0x00, 0xC5, 0x8E, 0xB1, 0xE6, 0xC8, 0x0E, 0x93, 0x34 };
static const uint8_t PROGMEM APPSKEY[16] = { 0xFE, 0x5A, 0x6E, 0xB6, 0x3B, 0x0D, 0x2E, 0xAD, 0x17, 0xC0, 0x78, 0xFD, 0xF7, 0xD6, 0x80, 0xF9 };
static const uint32_t DEVADDR = 0x26011E27;


/* **************************************************************
 * user settings
 * *************************************************************/
// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const unsigned TX_INTERVAL = 600;  // not used
//unsigned int countero = 0;
unsigned int serial_act = 1;
unsigned long starttime;  //unsigned long cycle_length = TX_INTERVAL * 1000UL; // cycle in secs, currently unused;

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  120        /* Time ESP32 will go to sleep (in seconds) */
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int countero = 0;

#define ALT 44
#define COR (ALT/8.0)
unsigned int counter = 0; 


// data to send less is better!
static uint8_t dataTX[16];


// Uses LMIC libary by Thomas Telkamp and Matthijs Kooijman (https://github.com/matthijskooijman/arduino-lmic)
#include <lmic.h>
#include <hal/hal.h>

// Declare the job control structures
static osjob_t sendjob;

// These callbacks are only used in over-the-air activation, so they are
// left empty when ABP (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
    #ifdef ACT_METHOD_ABP
      void os_getArtEui (u1_t* buf) { }
      void os_getDevEui (u1_t* buf) { }
      void os_getDevKey (u1_t* buf) { }
    #else
      void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
      void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
      void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}
    #endif
/* ************************************************************** 
 * Pin mapping
 * *************************************************************/
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32},
};
/* ************************************************************** 
 * OLED setup
 * *************************************************************/
#include "SSD1306.h"

#define OLED_ADDR   0x3c  // OLED's ADDRESS
#define OLED_SDA  4
#define OLED_SCL  15
#define OLED_RST  16
SSD1306 display(OLED_ADDR, OLED_SDA, OLED_SCL);


/* ***************************************************************
* I2C scan BEBUG
* ****************************************************************/
void scanI2C(){
  
  byte error, address;
  int nDevices;
   Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
 
  delay(5000);           // wait 5 seconds for next scan

}

/* **************************************************************
 * SOUND LUX
 * *************************************************************/
float RawToLux(int raw){
float logLux = raw * luxlogRange / luxrawRange;
return pow(10, logLux);
}

void readSound(){
  int level = analogRead(soundPin);
  int *plevel = &level;
  #ifdef DEBUG
  // Print some messages
  Serial.print("Analog: ");
  Serial.print(level);
  Serial.print(" (");
  #endif
  // Print absolute sound db from level value
  soundValue = (get_abs_db(plevel));

}

int luxValue = 0; // variable to store the value coming from the sensor
void luxSound(){
  // read the raw value from the sensor:
  int rawValue = analogRead(luxPin);
   #ifdef DEBUG
   Serial.print("Raw light = ");
   Serial.print(rawValue);
   Serial.print(" - Lux = ");
   Serial.println(RawToLux(rawValue));
   #endif

  LuxSensorValue = RawToLux(rawValue);
}

/* **************************************************************
 * sensor code, typical would be init_sensor(), do_sense(), build_data()
 * *************************************************************/
/* **************************************************************
 * init the sensor
 * *************************************************************/
void init_sensor() {

//scanI2C();
  if(!ccs.begin(0x5A)){
    Serial.println("Failed to start Co2 sensor! Please check your wiring.");
    while(1);
  }

  if (!bme.begin(0x76)){
    Serial.println("Failed to start BME2180 sensor! Please check your wiring.");
    while(1);
  }

  //calibrate temperature sensor
  while(!ccs.available());
  float temp = ccs.calculateTemperature();
  ccs.setTempOffset(temp - 25.0);

  // SOUND
  pinMode(soundPin,INPUT);
  adcAttachPin(soundPin);

 
}


void show_display_A(){

  if (serial_act == 1) { 
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);

  String tempS = "Temperatura : " + String(tempC-10.0) + "°C";
  display.drawString(0, 1, tempS);
  String humidityS = "Humidade : " + String(humidity) + "%";
  display.drawString(0, 12, humidityS);
  //u8x8.setCursor(0, 5);
  // u8x8.printf("Press: %.1fhPa", pressure);
  String presS = "Pressão : " + String(pressure) + "hPa";
  display.drawString(0, 23, presS);
  String alt = "Altitude : " + String(altitudeMeter) + "m";
  display.drawString(0, 34, alt);
  String counters = "Pacotes Enviados : " + String(countero);
  display.drawString(0, 45, counters);
  display.display();
  }
}
void show_display_B(){

  if (serial_act == 1) { 
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);

  String eCo2S = "eCo2 : " + String(eCO2) + "";
  display.drawString(0, 1, eCo2S);
  String TVOCS = "TVOC : " + String(TVOC) + "ppm";
  display.drawString(0, 12, TVOCS);
  //u8x8.setCursor(0, 5);
  // u8x8.printf("Press: %.1fhPa", pressure);
  String soundSS = "Sound : " + String(soundValue) + "dB";
  display.drawString(0, 23, soundSS);
  String luxS = "Luminosidade : " + String(LuxSensorValue) + "lux";
  display.drawString(0, 34, luxS);
  String counters = "Pacotes Enviados : " + String(countero);
  display.drawString(0, 45, counters);
  display.display();
  }
}

/* **************************************************************
 * do the reading
 * *************************************************************/
void do_sense() {

        // SOUND LUX DEBUG
   readSound();
   luxSound();

  // BME -----------------------------------
  tempC = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;
  altitudeMeter = bme.readAltitude(SEALEVELPRESSURE_HPA);


  if (isnan(pressure) || isnan(tempC) || isnan(humidity) || isnan(altitudeMeter)) {
    #ifdef DEBUG   
      Serial.println("Failed to read from BME280 sensor!");
    #endif  
    return;
  }
  
  show_display_A();


  // Co2 ------ BURN SENSOR ----------------------
  for(int i = 0; i <= 11; i++ ){
     
    Serial.print(".");
      eCO2 = ccs.geteCO2();
      delay(500);
      TVOC = ccs.getTVOC();
      delay(500);
      if (isnan(eCO2) || isnan(TVOC) ) {
        #ifdef DEBUG   
          Serial.println("Failed to read from Co2 sensor!");
        #endif  
        return;
      }

    
   // #ifdef DEBUG // -- DEBUG ----  
    if(!ccs.readData()){ 
      Serial.print("eCO2: ");
      Serial.print(eCO2);
      Serial.print(" ppm, TVOC: ");      
      Serial.print(TVOC);
      Serial.println("");
    }
  //  #endif
    //----------------  
  delay(10000); // time for co2 burn
  if (i ==6){
     show_display_B();
  }

}



  #ifdef DEBUG
    Serial.print(F(" temp Celcius:"));
    Serial.print(tempC);
    Serial.print(F(" humidity:"));
    Serial.print(humidity);
    Serial.print(F(" pressure:"));
    Serial.print(pressure);
    Serial.print(F(" altitude (meters):"));
    Serial.print(altitudeMeter);    
    Serial.println(F(""));
  #endif
  delay(2000);
}

/* **************************************************************
 * build data to transmit in dataTX
 *
 * Sugestão para Payload
 *
 * function Decoder(bytes, port) {
 *  var temp = parseInt(bytes[0] + (bytes[1] << 8 ) - 500) / 10;
 *  var humidity = parseInt(bytes[2] + (bytes[3] << 8 ));
 *  var pressure = parseInt(bytes[4] + (bytes[5] << 8 )) / 10;
 *  var altitude = parseInt(bytes[6] + (bytes[7] << 8 ) - 100) / 10;
 *  var eCo2 = parseInt(bytes[8] + (bytes[9] << 8 ) ) / 10; 
 *  var TVOC = parseInt(bytes[10] + (bytes[11] << 8 )) / 10; 
 *  var LUX = parseInt(bytes[12] + (bytes[13] << 8 )) ; 
 *  var SOUND = parseInt(bytes[14] + (bytes[15] << 8 )) / 100;  
 *   
 *  return { temp: temp,
 **         humidity: humidity,
 *         pressure: pressure, 
 *         altitude: altitude,
 *         eCo2:eCo2,
 *         TVOC:TVOC,
 *         LUX:LUX,
 *         SOUND:SOUND
 *  }
 * }
 * 
 * *************************************************************/
void build_data() {

  int dtempC = (tempC+40) * 10;
  int dpressure = pressure * 10;
  int daltitude = (altitudeMeter)  * 10;

  int dco2 = (eCO2)  * 10;
  int dTVOC = (TVOC)  * 10;

  int dSoundValue = soundValue *10;

  dataTX[0] = dtempC;
  dataTX[1] = dtempC >> 8;
  dataTX[2] = int(humidity);
  dataTX[3] = int(humidity) >> 8;
  dataTX[4] = dpressure;
  dataTX[5] = dpressure >> 8;
  dataTX[6] = daltitude;
  dataTX[7] = daltitude >> 8;  

  dataTX[8]  = (dco2);
  dataTX[9]  = (dco2) >> 8;  
  dataTX[10] = (dTVOC);
  dataTX[11] = (dTVOC) >> 8;  

  dataTX[12] = LuxSensorValue;
  dataTX[13] = (LuxSensorValue) >> 8;

  dataTX[14] = dSoundValue;
  dataTX[15] = (dSoundValue) >> 8;

Serial.println(dtempC);
}

/* **************************************************************
 * radio code, typical would be init_node(), do_send(), etc
 * *************************************************************/
/* **************************************************************
 * init the Node
 * *************************************************************/
void init_node() {
  #ifdef VCC_ENABLE
     // For Pinoccio Scout boards
     pinMode(VCC_ENABLE, OUTPUT);
     digitalWrite(VCC_ENABLE, HIGH);
     delay(1000);
  #endif

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  #ifdef ACT_METHOD_ABP
    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
      // On AVR, these values are stored in flash and only copied to RAM
      // once. Copy them to a temporary buffer here, LMIC_setSession will
      // copy them into a buffer of its own again.
      uint8_t appskey[sizeof(APPSKEY)];
      uint8_t nwkskey[sizeof(NWKSKEY)];
      memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
      memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
      LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
      // If not running an AVR with PROGMEM, just use the arrays directly
      LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    // DAEY single CHANNEL gateway
        #define CHANNEL  1
        for (uint8_t i = 0; i < 9; i++) {
            if (i != CHANNEL) {
                LMIC_disableChannel(i);
            }
        }


    #if defined(CFG_eu868)
      // Set up the channels used by the Things Network, which corresponds
      // to the defaults of most gateways. Without this, only three base
      // channels from the LoRaWAN specification are used, which certainly
      // works, so it is good for debugging, but can overload those
      // frequencies, so be sure to configure the full frequency range of
      // your network here (unless your network autoconfigures them).
      // Setting up channels should happen after LMIC_setSession, as that
      // configures the minimal channel set.
      // NA-US channels 0-71 are configured automatically
    //LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    //LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
      // TTN defines an additional channel at 869.525Mhz using SF9 for class B
      // devices' ping slots. LMIC does not have an easy way to define set this
      // frequency and support for class B is spotty and untested, so this
      // frequency is not configured here.
    #elif defined(CFG_us915)
      // NA-US channels 0-71 are configured automatically
      // but only one group of 8 should (a subband) should be active
      // TTN recommends the second sub band, 1 in a zero based count.
      // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
      LMIC_selectSubBand(1);
    #endif

    #if defined(DEBUG)
      LMIC_disableChannel(1);
      LMIC_disableChannel(2);
      LMIC_disableChannel(3);
      LMIC_disableChannel(4);
      LMIC_disableChannel(5);
      LMIC_disableChannel(6);
      LMIC_disableChannel(7);
      LMIC_disableChannel(8);
    #endif
    
    // Enable data rate adaptation
    // LMIC_setAdrMode(1);

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);
  #endif

  #ifdef ACT_METHOD_OTAA
    // got this fix from forum: https://www.thethingsnetwork.org/forum/t/over-the-air-activation-otaa-with-lmic/1921/36
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
  #endif

}

/* *****************************************************************************
* send_message
* ****************************************************************************/
void send_message(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, dataTX, sizeof(dataTX), 0);
    Serial.println(F("Packet queued"));
  }
}

/*******************************************************************************/
void onEvent (ev_t ev) {
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.dataLen) {
        // data received in rx slot after tx
        Serial.print(F("Data Received: "));   
        Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
        Serial.println();
      }
      // schedule next transmission
      // os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), send_message);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
    
}


/* **************************************************************
 * send the message
 * *************************************************************/
void do_send() {
  
  Serial.print(millis());
  Serial.println(F(" Sending.. "));

  send_message(&sendjob);

  // wait for send to complete
  Serial.print(millis());
  Serial.print(F(" Waiting.. "));     
 
  while ( (LMIC.opmode & OP_JOINING) or (LMIC.opmode & OP_TXRXPEND) ) { os_runloop_once();  }
  Serial.print(millis());
  Serial.println(F(" TX_COMPLETE"));
  countero++;
}











void setup() {  
  Serial.begin(9600);                   // initialize serial
  while (!Serial);
  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  // Use the Blue pin to signal transmission.
  pinMode(LEDPIN,OUTPUT);

  pinMode(OLED_RST,OUTPUT);
  digitalWrite(OLED_RST, LOW); 
  delay(50);
  digitalWrite(OLED_RST, HIGH); 
  delay(5000);


  display.init();
  display.flipScreenVertically();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.clear();


  //--------------------- do stuff
  init_node();
  init_sensor();
  //scanI2C();
  // --------------------------------


  Serial.println("IPG SFarm LoRa V0.5");
  display.drawString(0, 00, "IPG SFarm LoRa V0.6");
  display.display();

  delay(1000);
   // starttime = millis();
    if(bootCount == 0) //Run this only the first time
      {
          bootCount = bootCount+1;
      }  
      Serial.print("init eCO2 .");
      do_sense();
      build_data();
      LMIC.seqnoUp += bootCount; // deep sleep guardar ounter pacotes
 
    // int analogValue = analogRead(37);////ANALOG_PIN);
    // float voltage = analogValue * (3.3 / ANALOG_MAX_VALUE);
    // Serial.println(analogValue);

     
     do_send();
 
  Serial.println("Going to sleep now");
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_deep_sleep_start();

//Serial.println("Bring ESP to sleep mode 5minutes");
//ESP.deepSleep(120e6);  /* 5e6 is 5x10^6 microseconds */

}

void loop() {


}
