/* 
*****************************
FIREWALL: SCOUT LITE
SOLAR POWERED DEVICE
V4
*****************************

*/

#include <Arduino.h>
#include <MKRWAN.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <ArduinoLowPower.h>
#include <Wire.h>
#include <LoRa.h>

//#include "ttn_config_secrets.h"
//#include "ttn_config_secrets_2.h"
#include "ttn_config_secrets_3.h"

#define PGOOD_PIN 1
#define CHG_PIN 2
#define CHARGE_SAMPLES 10
#define SAMPLE_DELAY 100
#define ERROR_FLAG 255
#define MAX_JOIN_ATTEMPTS 8
#define JOIN_RETRY_SLEEP_MINUTES 30
#define JOIN_RETRY_DELAY_MINUTES 2
#define LORA_CONNECTION_STABILIZATION_DELAY_SECONDS 5
#define ERROR_BLINK_COUNT 5

uint32_t DUTY_CYCLE_MINUTES = 10;

enum SolarStatus { OFF = 0, ACTIVE = 1 };
enum BatteryStatus { CHARGING = 0, FULL = 1, DRAINING = 2 };

class LoRaWAN {
public:
  LoRaModem modem;
  String appEui = SECRET_APP_EUI;
  String appKey = SECRET_APP_KEY;
  uint16_t packetCount = 0;

  void init();
  void handleDownlink();
  void sendData(byte* payload, size_t payloadSize);
};

class BME680Sensor {
public:
  Adafruit_BME680 bme;
  void init();
  void readAndSendData(LoRaWAN& lorawan);
};

class DeviceHealth {
public:
  SolarStatus solar_status;
  BatteryStatus battery_status;

  void update();
};

// Declare objects globally
LoRaWAN lorawan;
BME680Sensor sensor;
DeviceHealth deviceHealth;

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PGOOD_PIN, INPUT);
  pinMode(CHG_PIN, INPUT);

  Wire.begin();

  lorawan.init();
  sensor.init();
}

void loop() {
  sensor.readAndSendData(lorawan);
  lorawan.handleDownlink();
  LowPower.sleep(DUTY_CYCLE_MINUTES * 60 * 1000);
}

void LoRaWAN::init() {
  int joinAttempts = 0;
  if (!modem.begin(EU868)) {
    Serial.println("Failed to start module");
    while (1) {}
  }

  Serial.print("Your module version is: ");
  Serial.println(modem.version());
  Serial.print("Your device EUI is: ");
  Serial.println(modem.deviceEUI());

  int connected = modem.joinOTAA(appEui, appKey);

  while (!connected && joinAttempts < MAX_JOIN_ATTEMPTS) {
    joinAttempts++;
    Serial.println("Not connected, trying again...");
    delay(JOIN_RETRY_DELAY_MINUTES * 60 * 1000);  // wait before next attempt
    connected = modem.joinOTAA(appEui, appKey);
  }

  if (connected) {
    Serial.println("Network joined!");
  } else {
    Serial.println("Failed to join network after several attempts. The device is going into a low power mode before trying again.");
    LowPower.sleep(JOIN_RETRY_SLEEP_MINUTES * 60 * 1000);  // sleep
    init();  // try to initialize the LoRaWAN connection again
  }

  modem.setADR(true);
  modem.minPollInterval(60);
  delay(LORA_CONNECTION_STABILIZATION_DELAY_SECONDS * 1000);  // wait to stabilize connection
}

void LoRaWAN::handleDownlink() {
  String receivedData;
  int i = 0;
  while (modem.available()) {
    receivedData += (char)modem.read();
    i++;
  }

  if (i == 0) {
    Serial.println("No downlink data received");
    return;
  }

  Serial.println("Received downlink data: " + receivedData);

  // If the received data is the "EXTRA_MEASURE" command, take an extra measurement
  if (receivedData == "EXTRA_MEASURE") {
    Serial.println("Extra measurement command received. Taking extra measurement...");
    BME680Sensor sensor;
    sensor.readAndSendData(*this);
  } else {
    // If the received data is a number, use it to set the duty cycle
    for (char c : receivedData) {
      if (!isDigit(c)) {
        Serial.println("Invalid data received for duty cycle. Ignoring...");
        return;
      }
    }
    DUTY_CYCLE_MINUTES = receivedData.toInt();  
  }
}

void LoRaWAN::sendData(byte* payload, size_t payloadSize) {
  int err;
  modem.beginPacket();
  modem.write(payload, payloadSize);
  err = modem.endPacket(true);

  if (err > 0) {
    packetCount++;
    Serial.println("LoRa Packet sent");
  } else {
    Serial.println("Error sending LoRa Packet");
  }
}

void BME680Sensor::init() {
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    errorLed(ERROR_BLINK_COUNT);
    LoRaWAN lorawan;
    String errorMessage = "Error: Could not find a valid BME680 sensor.";
    byte payload[errorMessage.length() + 1];
    payload[0] = ERROR_FLAG;
    memcpy(payload + 1, errorMessage.c_str(), errorMessage.length());
    lorawan.sendData(payload, sizeof(payload));
    LowPower.sleep(DUTY_CYCLE_MINUTES * 60 * 1000);
  }

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_8X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);

  Serial.println("BME680 sensor initialized and calibrated.");
}

void BME680Sensor::readAndSendData(LoRaWAN& lorawan) {
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    Serial.println("Failed to begin reading");
    return;
  }

  DeviceHealth deviceHealth;
  deviceHealth.update();

  if (!bme.endReading()) {
    Serial.println("Failed to complete reading");
    return;
  }

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
  payload[8] = (deviceHealth.battery_status << 2) | deviceHealth.solar_status;
  payload[9] = highByte(lorawan.packetCount);
  payload[10] = lowByte(lorawan.packetCount);

  lorawan.sendData(payload, sizeof(payload));
}

void DeviceHealth::update() {
  int pgood_count = 0;
  int chg_count = 0;

  for (int i = 0; i < CHARGE_SAMPLES; i++) {
    pgood_count += digitalRead(PGOOD_PIN) == LOW ? 1 : 0;
    chg_count += digitalRead(CHG_PIN) == LOW ? 1 : 0;
    delay(SAMPLE_DELAY);
  }

  if (pgood_count > CHARGE_SAMPLES / 2) {
    solar_status = ACTIVE;
    Serial.println("Solar panel is ACTIVE.");
  } else {
    solar_status = OFF;
    Serial.println("Solar panel is OFF.");
  }

  if (chg_count > CHARGE_SAMPLES / 2) {
    battery_status = CHARGING;
    Serial.println("Battery is CHARGING.");
    if (solar_status == OFF) {
      solar_status = ACTIVE;
      Serial.println("Correction: Solar panel is ACTIVE.");
    }
  } else {
    if (solar_status == ACTIVE) {
      battery_status = FULL;
      Serial.println("Battery is FULL.");
    } else {
      battery_status = DRAINING;
      Serial.println("Battery is DRAINING.");
    }
  }
  Serial.println();
}

void errorLed(int blinkCount) {
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < blinkCount; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}