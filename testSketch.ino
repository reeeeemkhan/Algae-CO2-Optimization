#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <BH1750.h>
#include <ESP32Servo.h>

// DS18B20 temperature sensor
#define ONE_WIRE_BUS 4

// Fan MOSFET control
#define FAN_PIN 13

// LED light
#define LIGHT_PIN 26

// LED brightness
// FINAL LIGHT LEVELS
const int LIGHT_LOW = 30;
const int LIGHT_MEDIUM = 90;
const int LIGHT_HIGH = 128;

// Start reactor at HIGH
int lightPWM = LIGHT_HIGH;


// FINAL AIRFLOW LEVELS
const int AIR_HIGH = 0;
const int AIR_MEDIUM = 90;
const int AIR_LOW = 180;

// Start reactor at MEDIUM airflow
int airflowPosition = AIR_MEDIUM;

const int airflowLevels[3] = {
  AIR_HIGH,
  AIR_MEDIUM,
  AIR_LOW
};

int currentAirIndex = 1; // start at medium

  bool fanTestMode = false;
  bool requestFanOn = false;
  bool thermalOverride = false;

  // ---- Optimizer ----

// Light choices
const int lightLevels[3] = {30, 90, 128};
int currentLightIndex = 2;    // Start HIGH

// Timing
const unsigned long SETTLE_TIME = 120000;   // 2 minutes
const unsigned long MEASURE_TIME = 180000;  // 3 minutes
const unsigned long SAMPLE_TIME = 10000;    // CO2 reading every 10 sec

// Improvement must be larger than normal sensor noise
const float IMPROVEMENT_MARGIN = 5.0;

// Previous/best performance
float bestScore = -1000;
int bestLightIndex = 2;

  // optimization control
  const bool RUN_OPTIMIZATION = false;

  bool optimizationFinished = false;

  // CO2 Performance History
  float differenceHistory[18];
  int historyCount = 0;
  int historyPosition = 0;


// CO2 sensors
HardwareSerial CO2Sensor1(2);
HardwareSerial CO2Sensor2(1);

const int CO2_1_RX = 16;
const int CO2_1_TX = 17;

const int CO2_2_RX = 32;
const int CO2_2_TX = 33;

// Sensor #1 reads approximately 12 ppm higher
const int CO2_1_OFFSET = 12;

// Command to request CO2 reading from MH-Z19C
byte readCO2Command[9] = {
  0xFF, 0x01, 0x86,
  0x00, 0x00, 0x00,
  0x00, 0x00, 0x79
};

Servo airServo;
const int SERVO_PIN = 25;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

BH1750 lightMeter;

void setup() {
  Serial.begin(115200);

 //LED PWM
  ledcAttach(LIGHT_PIN, 1200, 8);
  ledcWrite(LIGHT_PIN, lightPWM);


  //Temp sensor
  sensors.begin();

  //Fan
  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW);





  //Light sensor
  Wire.begin(21,22); //SDA = GPIO21, SCL = GPIO22

  //Servo (Air valve)
  airServo.setPeriodHertz(50);
  airServo.attach(SERVO_PIN, 500, 2400);
  airServo.write(airflowPosition);

  // CO2 sensors
  CO2Sensor1.begin(9600, SERIAL_8N1, CO2_1_RX, CO2_1_TX);
  CO2Sensor2.begin(9600, SERIAL_8N1, CO2_2_RX, CO2_2_TX);

  if (lightMeter.begin()) {
    Serial.println("BH1750 connected");
  } else {
    Serial.println("BH1750 NOT FOUND");
  }

  Serial.println("System started");
}

int readCO2(HardwareSerial &sensor) {

  while (sensor.available()) {
    sensor.read();
  }

  sensor.write(readCO2Command, 9);

  byte response[9];

  if (sensor.readBytes(response, 9) != 9) {
    return -1;
  }

  if (response[0] != 0xFF || response[1] != 0x86) {
    return -1;
  }

  byte checksum = 0;

  for (int i = 1; i < 8; i++) {
    checksum += response[i];
  }

  checksum = 0xFF - checksum + 1;

  if (response[8] != checksum) {
    return -1;
  }

  return response[2] * 256 + response[3];
}


// TEMP/FAN CONTROL

void updateFan() {

  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);

  // Check for temperature sensor failure
  if (tempC == DEVICE_DISCONNECTED_C || isnan(tempC)) {

    digitalWrite(FAN_PIN, HIGH);
    thermalOverride = true;

    Serial.println("TEMPERATURE SENSOR ERROR");
    Serial.println("FAN FORCED ON");

    return;
  }

  // Temperature safety override
  if (tempC >= 28.0) {
    thermalOverride = true;
  }
  else if (tempC <= 27.0) {
    thermalOverride = false;
  }

  bool fanOn;

  if (thermalOverride) {

    // Safety always takes priority
    fanOn = true;

  }
  else if (fanTestMode) {

    // Allow optimizer to control fan
    fanOn = requestFanOn;

  } else {

    // Original automatic temperature control
    fanOn = false;

  }

  digitalWrite(FAN_PIN, fanOn ? HIGH : LOW);

  Serial.print("Temperature: ");
  Serial.print(tempC);
  Serial.print(" C | Fan: ");
  Serial.println(fanOn ? "ON" : "OFF");

  if (thermalOverride) {
    Serial.println("TEMPERATURE SAFETY OVERRIDE ACTIVE");
  }
}

  void setFanMode(bool turnOn) {

  fanTestMode = true;

  requestFanOn = turnOn;

  Serial.print("Optimizer requested fan: ");

  if (turnOn) {
    Serial.println("ON");
  }
  else {
    Serial.println("OFF");
  }

  updateFan();
}

float measureCO2Score() {

  Serial.println("MEASURING CO2 PERFORMANCE...");

  unsigned long startTime = millis();

  float totalDifference = 0;
  int goodReadings = 0;

  while (millis() - startTime < MEASURE_TIME) {
    updateFan();

    int co2_1 = readCO2(CO2Sensor1);
    delay(200);

    int co2_2 = readCO2(CO2Sensor2);

    if (co2_1 != -1 && co2_2 != -1) {

      int correctedCO2_1 = co2_1 - CO2_1_OFFSET;

      int difference = correctedCO2_1 - co2_2;

      totalDifference += difference;
      goodReadings++;

      Serial.print("CO2 IN: ");
      Serial.print(correctedCO2_1);

      Serial.print(" | CO2 OUT: ");
      Serial.print(co2_2);

      Serial.print(" | Difference: ");
      Serial.println(difference);
    }

    delay(SAMPLE_TIME);
  }

  if (goodReadings == 0) {
    Serial.println("NO VALID CO2 DATA");
    return -1000;
  }

  float averageDifference = totalDifference / goodReadings;

  Serial.print("AVERAGE CO2 UPTAKE SCORE: ");
  Serial.println(averageDifference);

  return averageDifference;
}

void waitForSystemToSettle() {

  Serial.println("SYSTEM SETTLING...");

  unsigned long startTime = millis();

  while (millis() - startTime < SETTLE_TIME) {

    updateFan();

    delay(5000);
  }
}

void setLightLevel(int index) {

  currentLightIndex = index;

  lightPWM = lightLevels[index];

  ledcWrite(LIGHT_PIN, lightPWM);

  Serial.print("NEW LIGHT PWM: ");
  Serial.println(lightPWM);
}

void setAirflowLevel(int index) {

  if (index < 0 || index > 2) {
    return;
  }

  currentAirIndex = index;
  airflowPosition = airflowLevels[index];

  airServo.write(airflowPosition);

  Serial.print("Airflow changed to: ");

  if (index == 0) {
    Serial.println("HIGH");
  }
  else if (index == 1) {
    Serial.println("MEDIUM");
  }
  else {
    Serial.println("LOW");
  }
}

void optimizeLight() {

  Serial.println();
  Serial.println("===== LIGHT OPTIMIZATION =====");

  // Measure current performance
  Serial.println("Measuring current setting...");

  waitForSystemToSettle();

  float oldScore = measureCO2Score();

  if(oldScore == -1000) {
    Serial.println("Baseline Failed - Optimization Cancelled");
    return;

  }

  int oldLightIndex = currentLightIndex;


  // Choose another light level
  int testLightIndex;

  if (currentLightIndex == 2) {
    testLightIndex = 1;        // HIGH -> MEDIUM
  }
  else {
    testLightIndex = currentLightIndex + 1;
  }


  Serial.println("Testing new light level...");

  setLightLevel(testLightIndex);

  waitForSystemToSettle();

  float newScore = measureCO2Score();

  if(newScore == -1000) {
    Serial.println("Test Failed - Reverting Light");
    setLightLevel(oldLightIndex);
    return;

  }


  Serial.print("OLD SCORE: ");
  Serial.println(oldScore);

  Serial.print("NEW SCORE: ");
  Serial.println(newScore);


  // Did it improve enough to matter?
  if (newScore > oldScore + IMPROVEMENT_MARGIN) {

    Serial.println("IMPROVED -> KEEP NEW SETTING");

    bestScore = newScore;
    bestLightIndex = testLightIndex;

  }

  else {

    Serial.println("NO IMPROVEMENT -> REVERT");

    setLightLevel(oldLightIndex);

    bestScore = oldScore;
    bestLightIndex = oldLightIndex;
  }

  Serial.println("==============================");
}


void loop() {

  // Run one optimization test if selected
  if (RUN_OPTIMIZATION && !optimizationFinished) {

    optimizeLight();

    optimizationFinished = true;

    Serial.println("OPTIMIZATION FINISHED");
    Serial.println("CONTINUING TO MONITOR REACTOR");
  }

  // Temperature and fan
  updateFan();

  float tempC = sensors.getTempCByIndex(0);

  // Light measurement
  float lux = lightMeter.readLightLevel();

  // CO2 measurements
  int co2_1 = readCO2(CO2Sensor1);

  delay(200);

  int co2_2 = readCO2(CO2Sensor2);

  if (co2_1 != -1 && co2_2 != -1) {

    int correctedCO2_1 = co2_1 - CO2_1_OFFSET;

    int difference = correctedCO2_1 - co2_2;

    // Store this measurement in our history
    differenceHistory[historyPosition] = difference;

    historyPosition = (historyPosition + 1) % 18;

    if (historyCount < 18) {
      historyCount++;
    }

    // Calculate rolling average
    float sum = 0;

    for (int i = 0; i < historyCount; i++) {
      sum += differenceHistory[i];
    }

    float averageDifference = sum / historyCount;

    // Print readings
    Serial.println();
    Serial.println("====== REACTOR DATA ======");

    Serial.print("Time: ");
    Serial.print(millis() / 1000);
    Serial.println(" seconds");

    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.println(" C");

    Serial.print("Light: ");
    Serial.print(lux);
    Serial.println(" lux");

    Serial.print("LED PWM: ");
    Serial.println(lightPWM);

    Serial.print("Airflow servo angle: ");
    Serial.println(airflowPosition);

    Serial.print("CO2 IN: ");
    Serial.print(correctedCO2_1);
    Serial.println(" ppm");

    Serial.print("CO2 OUT: ");
    Serial.print(co2_2);
    Serial.println(" ppm");

    Serial.print("CO2 Difference: ");
    Serial.print(difference);
    Serial.println(" ppm");

    Serial.print("Rolling Average: ");
    Serial.print(averageDifference);
    Serial.println(" ppm");

    Serial.println("==========================");
  }

  else {
    Serial.println("CO2 SENSOR ERROR - DATA SKIPPED");
  }

  delay(10000);
}