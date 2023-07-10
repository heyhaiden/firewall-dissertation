/* 
*****************************
FIREWALL: SCOUT LITE
SOLAR POWERED DEVICE
V2
10/07/2023
*****************************

*/

#include <Arduino.h>
#include <MKRWAN.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <ArduinoLowPower.h>
#include <Wire.h>
#include <LoRa.h>

// TTN Configuration
//#include "ttn_config_secrets.h"
//#include "ttn_config_secrets_2.h"
//include "ttn_config_secrets_3.h"

String appEui = SECRET_APP_EUI;
String appKey = SECRET_APP_KEY;

LoRaModem modem;
Adafruit_BME680 bme;

#define PGOOD_PIN 1
#define CHG_PIN 2

enum SolarStatus { OFF = 0, ACTIVE = 1 };
enum BatteryStatus { CHARGING = 0, FULL = 1, DRAINING = 2 };

SolarStatus solar_status;
BatteryStatus battery_status;

uint16_t packetCount = 0;
uint32_t DUTY_CYCLE = 2 * 60 * 1000;
uint32_t SHORT_DUTY_CYCLE = 5 * 60 * 1000; 

void setup() {
  
  Serial.begin(115200);
  delay(1000);
  
  pinMode(PGOOD_PIN, INPUT);
  pinMode(CHG_PIN, INPUT);

  Wire.begin();

  initIaqSensor();
  initLoRaWan();

  //LowPower.attachInterruptWakeup(RTC_ALARM_WAKEUP, dummy, CHANGE);
}

void loop() {

  // Tell BME680 to begin measurement.
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    //Serial.println(F("Failed to begin reading :("));
    return;
  }
  //Serial.print(F("Reading started at "));
  //Serial.print(millis());
  //Serial.print(F(" and will finish at "));
  //Serial.println(endTime);

   //Serial.println(F("You can do other work during BME680 measurement."));
  //delay(50); // This represents parallel work.
  // There's no need to delay() until millis() >= endTime: bme.endReading()
  // takes care of that. It's okay for parallel work to take longer than
  // BME680's measurement time.

  // Obtain measurement results from BME680. Note that this operation isn't
  // instantaneous even if milli() >= endTime due to I2C/SPI latency.
  if (!bme.endReading()) {
    //Serial.println(F("Failed to complete reading :("));
    return;
  }
  //Serial.print(F("Reading completed at "));
  //Serial.println(millis());

  // if (!bme.performReading()) {
  //   Serial.println("Failed to perform reading :(");
  //   return;
  // }

  //bmeSensorRead();
  updateDeviceHealth();
  sendDataOverLoRaWAN();

  // Before sleeping, check the temperature and adjust the sleep time
  // if (bme.temperature >= 28.0) {
  //   modem.sleep(SHORT_DUTY_CYCLE);
  //   LowPower.deepSleep(SHORT_DUTY_CYCLE);
    
  // } else {
  //   modem.sleep(DUTY_CYCLE);
  //   LowPower.deepSleep(DUTY_CYCLE);  
  // }

  modem.sleep(DUTY_CYCLE);
  LowPower.sleep(DUTY_CYCLE);
  
}

// void dummy() {
//   // This function will be called once on device wakeup
//   // You can do some little operations here (like changing variables which will be used in the loop)
//   // Remember to avoid calling delay() and long running functions since this functions executes in interrupt context
//   if (!bme.begin(0x76)) {
//     //errLeds();
//     while (1)
//       ;
//   }
// }

void initSensor() {

  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1)
      ;
  }

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_8X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);

  Serial.println("BME680 sensor initialized and calibrated.");
}

void initLoRaWan() {
  int joinAttempts = 0;
  int maxJoinAttempts = 4;
  uint32_t sleepPeriod = 30 * 60 * 1000;  // sleep for 30 minutes, in milliseconds

  if (!modem.begin(EU868)) {
    Serial.println("Failed to start module");
    while (1) {}
  }

  Serial.print("Your module version is: ");
  Serial.println(modem.version());
  Serial.print("Your device EUI is: ");
  Serial.println(modem.deviceEUI());

  int connected = modem.joinOTAA(appEui, appKey);

  while (!connected && joinAttempts < maxJoinAttempts) {
    joinAttempts++;
    Serial.println("Not connected, trying again...");
    delay(10000);  // wait 10 seconds before next attempt
    connected = modem.joinOTAA(appEui, appKey);
  }

  if (connected) {
    Serial.println("Network joined!");
  } else {
    Serial.println("Failed to join network after several attempts. The device is going into a low power mode for 1 hour before trying again.");
    LowPower.sleep(sleepPeriod);  // sleep for 1 hour
    initLoRaWan();                // try to initialize the LoRaWAN connection again
  }

  modem.setADR(true);
  modem.minPollInterval(60);
  delay(5000);  // wait 5 seconds to stabilize connection
}

void bmeSensorRead() {
  // Print the sensor readings to the console
  Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
  Serial.println(" %");

  Serial.print("Gas Resistance = ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(" KOhms");

  Serial.println();
}

void updateDeviceHealth() {

  if (digitalRead(PGOOD_PIN) == LOW) {
    solar_status = ACTIVE;
    //Serial.println("Solar panel is ACTIVE.");
  } else {
    solar_status = OFF;
    //Serial.println("Solar panel is OFF.");
  }

  if (digitalRead(CHG_PIN) == LOW) {
    battery_status = CHARGING;
    //Serial.println("Battery is CHARGING.");
  } else {
    if (solar_status == ACTIVE) {
      battery_status = FULL;
      //Serial.println("Battery is FULL.");
    } else {
      battery_status = DRAINING;
      //Serial.println("Battery is NOT CHARGING and solar is OFF, so battery status may not be FULL.");
    }
  }
}

void sendDataOverLoRaWAN() {
  int temperature = bme.temperature * 100.0;
  int humidity = bme.humidity * 100.0;
  int pressure = 10 * (bme.pressure / 100.0);
  int gas_resistance = bme.gas_resistance / 10.0;

  byte payload[11];
  payload[0] = highByte(temperature);
  payload[1] = lowByte(temperature);
  payload[2] = highByte(humidity);
  payload[3] = lowByte(humidity);
  payload[4] = highByte(pressure);
  payload[5] = lowByte(pressure);
  payload[6] = highByte(gas_resistance);
  payload[7] = lowByte(gas_resistance);
  payload[8] = (battery_status << 2) | solar_status; // switched values to give priority to battery_status 3 states; values were getting incorrectly reported when solar was off and battery draining
  payload[9] = highByte(packetCount);
  payload[10] = lowByte(packetCount);

  int err;
  modem.beginPacket();
  modem.write(payload, sizeof(payload));
  err = modem.endPacket(true);

  if (err > 0) {
    packetCount++;
    Serial.println("LoRa Packet sent");
  } else {
    Serial.println("Error sending LoRa Packet");
  }
}

void errLeds(void) {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}

void shortLeds(void) {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}