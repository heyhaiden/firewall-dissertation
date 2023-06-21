/*
  FireWall
  v1.0
  16/05/2023
*/

#include <MKRWAN.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <ArduinoLowPower.h>
#include <bsec.h>
#include "arduino_secrets.h"

String appEui = SECRET_APP_EUI;
String appKey = SECRET_APP_KEY;

LoRaModem modem;
Adafruit_BME680 bme;

void setup() {
  Serial.begin(9600);

  Wire.begin();

  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); 

  Serial.println("BME680 sensor initialized and calibrated.");

  if (!modem.begin(EU868)) {
    Serial.println("Failed to start module");
    while (1) {}
  }

  Serial.print("Your module version is: ");
  Serial.println(modem.version());
  Serial.print("Your device EUI is: ");
  Serial.println(modem.deviceEUI());

  int connected = modem.joinOTAA(appEui, appKey);
  if (!connected) {
    Serial.println("Something went wrong; are you indoor? Move near a window and retry");
    while (1) {}
  }

  modem.setADR(true);
  modem.minPollInterval(60);
}

void loop() {
   // Take a reading from the BME680 sensor
  if (!bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

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

  
  //data.device_status = getDeviceStatus(); // replace with your function to get device status
  //data.battery_health = getBatteryHealth(); // replace with your function to get battery health


  int temperature = 100*bme.temperature;
  int pressure = 10*(bme.pressure/100.0); 
  int humidity = 100*bme.humidity;
  int gas_resistance = 100*(bme.gas_resistance/1000.0);
  
  // Send LoRaWAN packet
  byte payload[8];
  payload[0] = highByte (temperature);
  payload[1] = lowByte (temperature);
  payload[2] = highByte (pressure);
  payload[3] = lowByte (pressure);
  payload[4] = highByte (humidity);
  payload[5] = lowByte (humidity);
  payload[6] = highByte (gas_resistance);
  payload[7] = lowByte (gas_resistance);
 
  int err;
  modem.beginPacket();
  modem.write(payload, sizeof(payload));
  err = modem.endPacket(true);

  if (err > 0) {
    Serial.println("LoRa Packet sent successfully!");
  } else {
    Serial.println("Failed to send LoRa Packet :(");
  }

  // Sleep for 2 minutes to conserve battery life
  //LowPower.deepSleep(2 * 60 * 1000);  // Sleep for 2 minutes
  delay(20000);
}
