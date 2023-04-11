#include <Arduino.h>
// Fix Parameters
// Possible Values for Serial_Print_Mode  ! DONT TOUCH !
//
// Skatch to measure the outdoor termperature with a DS1820 onewire temp sensor
// send the temperature via TTN and forward it to my home server
//

#define   Serial_None            0  // No Serial Printout
#define   Serial_Debug           1  // Only debug and error output will be printed via RS232(USB)
#define   Serial_Logging         2  // Log measurement as table via RS232(USB)
#define   Serial_One_Minute_Log  3  // One Minute logging will be printed via RS232(USB)
#define   Serial_Statistics_Log  4  // Lists time between two events in us via RS232(USB)

// Usable CPU-Types
// WIFI -> Heltev Wifi Kit 32
#define WIFI 0
// LORA  ->  Heltec Wifi Lora 32 (V2)
#define LORA 1
// STICK ->  Heltec Wireless Stick  (has LoRa on board)
#define STICK 2
//
// Includes
//====================================================================================================================================
#include "userdefines.h"
//==================================================================================================================================
#include <Arduino.h>
#include <U8x8lib.h>

char          ssid[30];
float currentVoltage=0;
uint8_t tx_payload[18];

#include "thp_sensor.h"
//==================================================================================================================================
// inclue Lora support
//==================================================================================================================================
#include "loraWan.h"
#include "display.h"
//==================================================================================================================================
// GPS
// 
//==================================================================================================================================
#include "RTClib.h"
#include <TinyGPS++.h>

// TTN Send message interval (default 5min = 300 sec)) [sec]
#define TTN_MESSAGING_INTERVAL  30 // in seconds
static unsigned long transmission_timestamp = millis(); 

//==================================================================================================================================
// rtc support - however currently not working 
//==================================================================================================================================
DateTime now;
RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};


void read_THP(unsigned long current_ms,
              bool *have_thp, float *temperature, float *humidity, float *pressure) {
  static unsigned long last_timestamp = 0;
  // first call: immediately query thp sensor
  // subsequent calls: only query every MEASUREMENT_INTERVAL

    *have_thp = read_thp_sensor(temperature, humidity, pressure);

}
//==================================================================================================================================
// Define the GPS Interface
//==================================================================================================================================
#define RXD2 23
#define TXD2 17
HardwareSerial neogps(2);  //?? Txt 2
 
TinyGPSPlus gps;

double lat, lng, alt, speed, course;
uint32_t sat;

//==================================================================================================================================
// get the chip id 
//==================================================================================================================================
unsigned long getESPchipID() {
  uint64_t espid = ESP.getEfuseMac();
  uint8_t *pespid = (uint8_t*)&espid;
  uint32_t id = 0;
  uint8_t *pid = (uint8_t *)&id;
  pid[0] = (uint8_t)pespid[5];
  pid[1] = (uint8_t)pespid[4];
  pid[2] = (uint8_t)pespid[3];
  Serial.printf("ID: %08X\n", id);
  Serial.printf("MAC: %04X%08X\n",(uint16_t)(espid>>32),(uint32_t)espid);
  return id;
}

//
//
//
void generate_payload(float temperature, float humidity, float pressure,  float currentVoltage ) {
  uint32_t temp_Binary = temperature * 100;
  uint32_t humidityBinary = humidity * 100;
  uint32_t pressureBinary = pressure * 100;
  
  uint32_t currentVoltage_Binary = ((currentVoltage) * 1000);

  uint8_t payload[18];

  
  payload[0] = ( temp_Binary >> 8 ) & 0xFF;
  payload[1] = temp_Binary & 0xFF;

  payload[2] = ( humidityBinary >> 8 ) & 0xFF;
  payload[3] = humidityBinary & 0xFF;

  payload[4] = ( pressureBinary >> 8 ) & 0xFF;
  payload[5] = pressureBinary & 0xFF;

  payload[6] = (currentVoltage_Binary >> 8) & 0xFF;
  payload[7] = currentVoltage_Binary  & 0xFF;
  
  
  int i = 0;
  while (i < sizeof(payload)) {
    tx_payload[i] = payload[i];
    i++;
  }
}
//==================================================================================================================================
// LoRa payload:
// to minimise airtime, we only send necessary bytes. We do NOT use Cayenne LPP.
// The payload will be translated via http integration and a small python program
// to be compatible with luftdaten.info. For byte definitions see ttn2luft.pdf in
// docs directory
//==================================================================================================================================
void sendData2TTN(int event, int tbd, unsigned int temperature) {
  Serial.println("in sendData2TTN");
  Serial.print("temperature = ");
  Serial.println(temperature);
  Serial.println("data buffer to be sent = ");
  unsigned char ttnData[20];
  for (int i=0; i < 20; i++) {
     ttnData[i] = 0x00;
  };
  
  ttnData[0] = event;
  ttnData[1] = tbd;
  ttnData[2] = (byte) ((temperature & 0xFF000000) >> 24);
  ttnData[3] = (byte) ((temperature & 0x00FF0000) >> 16);
  ttnData[4] = (byte) ((temperature & 0x0000FF00) >> 8);
  ttnData[5] = (byte) ((temperature & 0x000000FF));
  int cnt;
  cnt = 6;
  Serial.print("0= ");
  Serial.println(ttnData[0], HEX);
  Serial.print("1= ");
  Serial.println(ttnData[1], HEX);
  Serial.print("2= ");
  Serial.println(ttnData[2], HEX);
  Serial.print("3= ");
  Serial.println(ttnData[3], HEX);
  Serial.print("4= ");
  Serial.println(ttnData[4], HEX);
  Serial.print("5= ");
  Serial.println(ttnData[5], HEX);
  Serial.print("6= ");
  Serial.println(ttnData[6], HEX);
  Serial.println("");
  
  lorawan_send(1,ttnData,cnt,false,NULL,NULL,NULL);
}
//==================================================================================================================================
// send Data to TTN
//==================================================================================================================================
void sendBMEData2TTN(int event, int tbd, float temperature, float humidity, float pressure, float currentVoltage) {
 
  unsigned char ttnData[20];
  for (int i=0; i < 20; i++) {
     ttnData[i] = 0x00;
  };
  
  ttnData[0] = event;
  ttnData[1] = tbd;

  uint32_t temp_Binary = temperature * 100;
  uint32_t humidityBinary = humidity * 100;
  uint32_t pressureBinary = pressure * 100;
  
  uint32_t currentVoltage_Binary = ((currentVoltage) * 1000);

  uint8_t payload[18];

  
  ttnData[2] = ( temp_Binary >> 8 ) & 0xFF;
  ttnData[3] = temp_Binary & 0xFF;

  ttnData[4] = ( humidityBinary >> 8 ) & 0xFF;
  ttnData[5] = humidityBinary & 0xFF;

  ttnData[6] = ( pressureBinary >> 16 ) & 0xFF;
  ttnData[7] = ( pressureBinary >> 8 ) & 0xFF;
  ttnData[8] = pressureBinary & 0xFF;

  ttnData[9] = (currentVoltage_Binary >> 8) & 0xFF;
  ttnData[10] = currentVoltage_Binary  & 0xFF;
  
  int cnt = 12;
  lorawan_send(1,ttnData,cnt,false,NULL,NULL,NULL);
}
//****************************************************************************************************
//*** setup routine
//****************************************************************************************************
void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("Let's go!");
  uint32_t xx = getESPchipID();

  // build SSID
  sprintf(ssid,"ESP32-%d",xx);
  Serial.println(ssid);

  //==================================================================================================================================
  // start GPS modul
  //==================================================================================================================================
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
  
  //==================================================================================================================================
  Serial.println("calling setup_display");
  setup_display();
  delay(1000);
  //==================================================================================================================================
  // Initialize real time clock and ads1115
  //==================================================================================================================================
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    delay(1000);
  }
  //rtc.adjust(DateTime(2021, 1, 11, 18, 06, 0));
  now = rtc.now();
  Serial.print("now time = ");
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, lets set the time!");
    // following line sets the RTC to the date &amp; time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date &amp; time, for example to set
    // January 21, 2014 at 3am you would call:
   rtc.adjust(DateTime(2021, 1, 12, 11, 19, 0));
  }
  //==================================================================================================================================
  // init LoRa
  //==================================================================================================================================
  lorawan_setup();
  //==================================================================================================================================
  // init bme280 sensor
  //==================================================================================================================================
   setup_thp_sensor();
   // battery power
   adcAttachPin(37);
} // end of setup

//****************************************************************************************************
// main loop
//****************************************************************************************************
void loop()
{
  
  //===============================================================================================
  // read the Temperature value
  // calculate the integer value (get rid of the comma)
  //===============================================================================================
  
  unsigned long current_ms = millis();  // to save multiple calls to millis()
  static bool have_thp = true;
  static float temperature = 0.0, humidity = 0.0, pressure = 0.0;

  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n'); // read the incoming data until a newline character is received
    temperature = data.substring(0, data.indexOf(',')).toFloat(); // extract the first value before the comma delimiter and convert it to a float
    humidity = data.substring(data.indexOf(',') + 1).toFloat(); // extract the second value after the comma delimiter and convert it to a float
    //float pressure = data.substring(data.indexOf(',') + 2).toFloat(); // extract the second value after the comma delimiter and convert it to a float

    Serial.print("Value 1: ");
    Serial.println(temperature);
    Serial.print("Value 2: ");
    Serial.println(humidity);
    //Serial.print("Value 3: ");
    //Serial.println(pressure);
  } else {
    Serial.println("Error at Serial");
  }

  //read_THP(current_ms, &have_thp, &temperature, &humidity, &pressure);

  Serial.print("temperature ");
  Serial.println(temperature);
  Serial.print("humidity ");
  Serial.println(humidity);
  Serial.print("pressure ");
  Serial.println(pressure);
  display_thp(temperature, humidity, pressure); 
  
  Serial.println("-------");

  float vBat;
  float val = analogRead(37);
  vBat = val * (3.3/4095)  ;
  Serial.print("vBat ");
  Serial.println(vBat);
  //***************************************************************************************************
  //   loop on GPS serial communication - check if data is available every 1 sec new reading form GPS should occur
  //***************************************************************************************************
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;)
  { 
    while (neogps.available())
    {
      if (gps.encode(neogps.read()))
      {
        newData = true;
      }
    }
  }

  if(newData == true)
  {
    newData = false;

    if (gps.location.isValid() == 1) {
      Serial.println("gps data valid");
     
      lat=(gps.location.lat());
      lng=(gps.location.lng());
      alt=gps.altitude.meters();  
      sat=gps.satellites.value();
      course= gps.course.deg();
      //distbetween = gps.distanceBetween(gps.location.lat(), gps.location.lng(), landingLat, landingLng));
      speed=gps.speed.kmph();
      Serial.print("latitude = ");
      Serial.println(lat);
      Serial.print("longtitude = ");
      Serial.println(lng);
      Serial.print("altitude = ");
      Serial.println(alt);
      Serial.print("speed = ");
      Serial.println(speed);
      Serial.print("course = ");
      Serial.println(course);
      Serial.print("sat = ");
      Serial.println(sat);
    } else {
      Serial.println("gps data invalid");
      //display_gps(0, 0, 0, 0, 0 , 0);
    }
  }

  generate_payload(temperature,humidity,pressure,currentVoltage);

  // *************************************************************************************************
  // have to send the data? send in the interval of TTN_MESSAGING_INTERVAL
  // *************************************************************************************************
  if ((millis() - transmission_timestamp) >= (TTN_MESSAGING_INTERVAL * 1000)) {
    //DisplayGMC(100,200,300,true,false);
    transmission_timestamp=millis();
 
    Serial.println("Sending to TTN ...");
    // avoid neg. numbers on temperature - ttn payload formatter has to subtract the 500
    sendBMEData2TTN(1,0, (temperature+500), humidity, pressure, vBat);
    // print state of switch
    Serial.println("data sent");
  }
}