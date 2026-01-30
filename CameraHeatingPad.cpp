#include <Wire.h>
#include <SparkFun_MS5803_I2C.h>

// Create sensor object
MS5803 sensor(ADDRESS_HIGH);

// Heating pad control
const int HEATING_PAD_PIN = 7;
const float TEMP_THRESHOLD = 30;  // Temperature threshold in °C
const float HYSTERESIS = 0.5;
bool heatingPadState = false;

// Camera trigger
const int CAMERA_PIN = 4;      // Triggers photos being taken with camera
const float PHOTO_STEP = 100;  // Take a photo every 100 meters
float baseAltitude = 0;        // Reference altitude at start
int lastPhotoStepIndex = -1;   // Last 100 m index we have triggered on

// Altitude calculation constants
const float SEA_LEVEL_PRESSURE = 1013.25;

unsigned long flightStartTime = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Heating pad pin
  pinMode(HEATING_PAD_PIN, OUTPUT);
  digitalWrite(HEATING_PAD_PIN, LOW);

  // Camera pin
  pinMode(CAMERA_PIN, OUTPUT);
  digitalWrite(CAMERA_PIN, LOW);

  Serial.println("High Altitude Balloon Flight Computer");
  Serial.println("=====================================");

  // Reset and initialize the sensor
  sensor.reset();
  sensor.begin();

  // Give sensor a moment and grab an initial pressure reading for base altitude
  delay(200);
  float initTemp = sensor.getTemperature(CELSIUS, ADC_4096);
  float initPressure = sensor.getPressure(ADC_4096);
  baseAltitude = 44330.0 * (1.0 - pow(initPressure / SEA_LEVEL_PRESSURE, 0.1903));

  flightStartTime = millis();  // Record launch time

  Serial.println("Sensor initialized!");
  Serial.print("Base altitude (m): ");
  Serial.println(baseAltitude);
  Serial.println();
}

float calculateAltitude(float pressure, float seaLevelPressure) {
  return 44330.0 * (1.0 - pow(pressure / seaLevelPressure, 0.1903));
}

void triggerCamera() {
  Serial.println(">>> CAMERA TRIGGER <<<");
  // Simple rising pulse; adjust duration and polarity to match your GoPro trigger interface
  digitalWrite(CAMERA_PIN, HIGH);
  delay(200);  // 200 ms pulse; change if needed
  digitalWrite(CAMERA_PIN, LOW);
}

void loop() {
  // Get temperature in Celsius
  float temperature = sensor.getTemperature(CELSIUS, ADC_4096);

  // Get pressure in mbar
  float pressure = sensor.getPressure(ADC_4096);

  // Calculate altitude in meters (absolute)
  float altitude = calculateAltitude(pressure, SEA_LEVEL_PRESSURE);

  // Altitude relative to base
  float relativeAltitude = altitude - baseAltitude;

  unsigned long currentFlightTime = millis() - flightStartTime;

  // ===== HEATING PAD CONTROL =====
  if (!heatingPadState && temperature < TEMP_THRESHOLD - HYSTERESIS) {
    digitalWrite(HEATING_PAD_PIN, HIGH);
    heatingPadState = true;
    Serial.println(">>> HEATING PAD ON <<<");
  } 
  else if (heatingPadState && temperature > TEMP_THRESHOLD + HYSTERESIS) {
    digitalWrite(HEATING_PAD_PIN, LOW);
    heatingPadState = false;
    Serial.println(">>> HEATING PAD OFF <<<");
  }

  // ===== CAMERA TRIGGER EVERY 100 m =====
  // Only trigger for positive relative altitude
  if (relativeAltitude >= 0) {
    int currentStepIndex = (int)(relativeAltitude / PHOTO_STEP);  // 0 for 0–99.9 m, 1 for 100–199.9 m, etc.
    if (currentStepIndex != lastPhotoStepIndex) {
      // Entered a new 100 m band: trigger a photo
      triggerCamera();
      lastPhotoStepIndex = currentStepIndex;

      Serial.print("Photo step index: ");
      Serial.println(currentStepIndex);
      Serial.print("Relative altitude at photo: ");
      Serial.print(relativeAltitude);
      Serial.println(" m");
    }
  }

  // ===== PRINT STATUS =====
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" mbar");

  Serial.print("Altitude (absolute): ");
  Serial.print(altitude);
  Serial.print(" m (");
  Serial.print(altitude * 3.28084);
  Serial.println(" ft)");

  Serial.print("Altitude (relative to base): ");
  Serial.print(relativeAltitude);
  Serial.println(" m");

  Serial.print("Flight Time: ");
  Serial.print(currentFlightTime / 60000);
  Serial.println(" minutes");

  Serial.print("Heating Pad: ");
  Serial.println(heatingPadState ? "ON" : "OFF");

  Serial.println();

  delay(1000);
}
