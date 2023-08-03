/* 
Solar Charger State Reader
V3
*/

#define PGOOD_PIN 1
#define CHG_PIN 2

#define SAMPLES 10
#define SAMPLE_DELAY 100

enum SolarStatus { OFF = 0, ACTIVE = 1 };
enum BatteryStatus { CHARGING = 0, FULL = 1, DRAINING = 2 };

SolarStatus solar_status;
BatteryStatus battery_status;

void setup() {
  
  Serial.begin(115200);
  delay(1000);
  
  pinMode(PGOOD_PIN, INPUT);
  pinMode(CHG_PIN, INPUT);
  
}

void loop() {

  updateDeviceHealth();

  delay(10000);
  
}

void updateDeviceHealth() {

  int pgood_count = 0;
  int chg_count = 0;

  for (int i = 0; i < SAMPLES; i++) {
    pgood_count += digitalRead(PGOOD_PIN) == LOW ? 1 : 0;
    chg_count += digitalRead(CHG_PIN) == LOW ? 1 : 0;
    delay(SAMPLE_DELAY);
  }

  if (pgood_count > SAMPLES / 2) {
    solar_status = ACTIVE;
    Serial.println("Solar panel is ACTIVE.");
  } else {
    solar_status = OFF;
    Serial.println("Solar panel is OFF.");
  }

  if (chg_count > SAMPLES / 2) {
    battery_status = CHARGING;
    Serial.println("Battery is CHARGING.");
    // If the battery is charging, but the solar panel is reported as off, 
    // then the solar panel must actually be on. 
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
