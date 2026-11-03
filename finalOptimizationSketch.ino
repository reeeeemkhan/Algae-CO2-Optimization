#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <BH1750.h>
#include <ESP32Servo.h>
#include <math.h>

// ===================== OPERATING MODE =====================
// FALSE = fixed-settings control experiment
// TRUE = continuous optimization
const bool RUN_OPTIMIZATION = false;

// ===================== HARDWARE =====================
#define ONE_WIRE_BUS 4
#define FAN_PIN 13
#define LIGHT_PIN 26
#define SERVO_PIN 25

const int CO2_1_RX = 16;
const int CO2_1_TX = 17;
const int CO2_2_RX = 32;
const int CO2_2_TX = 33;

// Recheck this offset by exposing both sensors to the same air
const int CO2_1_OFFSET = 12;

// ===================== LIGHT / AIRFLOW =====================

// Light: LOW, MEDIUM, HIGH
const int lightLevels[3] = {30, 90, 128};

int currentLightIndex = 2;
int lightPWM = lightLevels[currentLightIndex];

// Airflow: HIGH, MEDIUM, LOW
const int airflowLevels[3] = {0, 90, 180};

int currentAirIndex = 1;
int airflowPosition = airflowLevels[currentAirIndex];

// Enter measured airflow values after calibration
const bool AIRFLOW_CALIBRATED = true;

const float airflowLpm[3] = { 
    0.223f,   // high
    0.144f,   // MEDIUM
    0.105f};  // low 

// ===================== TIMING =====================

// Provisional experimental durations
const unsigned long LIGHT_AIR_SETTLE_MS =
  2UL * 60UL * 1000UL;

const unsigned long LIGHT_AIR_MEASURE_MS =
  3UL * 60UL * 1000UL;

const unsigned long FAN_SETTLE_MS =
  45UL * 60UL * 1000UL;

const unsigned long FAN_MEASURE_MS =
  15UL * 60UL * 1000UL;

const unsigned long SAMPLE_MS = 10000UL;

const unsigned long CYCLE_PAUSE_MS = 60000UL;

// Must eventually validate against sensor noise
const float IMPROVEMENT_MARGIN_PPM = 5.0f;

// ===================== TEMPERATURE SAFETY =====================

const float FAN_FORCE_ON_C = 28.0f;
const float FAN_RELEASE_C = 27.0f;

const float MAX_TEMP_TO_START_FAN_TEST_C = 26.5f;

bool fanTestMode = false;
bool requestFanOn = false;
bool thermalOverride = false;

bool safetyTriggeredDuringTrial = false;

float lastTempC = NAN;
bool actualFanOn = false;

// ===================== SENSORS =====================

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

BH1750 lightMeter;
Servo airServo;

HardwareSerial CO2Sensor1(2);
HardwareSerial CO2Sensor2(1);

const byte readCO2Command[9] = {
  0xFF, 0x01, 0x86,
  0x00, 0x00, 0x00,
  0x00, 0x00, 0x79
};

// ===================== CO2 HISTORY =====================

float differenceHistory[18] = {0};

int historyCount = 0;
int historyPosition = 0;

// ===================== FAN CONTROL =====================

void updateFan() {

  sensors.requestTemperatures();

  lastTempC = sensors.getTempCByIndex(0);

  if (lastTempC == DEVICE_DISCONNECTED_C ||
      isnan(lastTempC)) {

    thermalOverride = true;
    safetyTriggeredDuringTrial = true;
    actualFanOn = true;

    digitalWrite(FAN_PIN, HIGH);

    Serial.println("TEMP SENSOR ERROR: FAN FORCED ON");

    return;
  }

  if (lastTempC >= FAN_FORCE_ON_C) {
    thermalOverride = true;
  }
  else if (lastTempC <= FAN_RELEASE_C) {
    thermalOverride = false;
  }

  if (thermalOverride) {
    safetyTriggeredDuringTrial = true;
  }

  actualFanOn = thermalOverride || (fanTestMode && requestFanOn);

  digitalWrite(FAN_PIN, actualFanOn ? HIGH : LOW);
}

// ===================== FAN MODE =====================

void setFanMode(bool on) {

  fanTestMode = true;
  requestFanOn = on;

  updateFan();

  Serial.print("Fan requested: ");
  Serial.println(on ? "ON" : "OFF");
}

// ===================== LIGHT CONTROL =====================

void setLightLevel(int index) {

  if (index < 0 || index > 2) {
    return;
  }

  currentLightIndex = index;

  lightPWM = lightLevels[index];

  ledcWrite(LIGHT_PIN, lightPWM);

  Serial.print("Light PWM: ");
  Serial.println(lightPWM);
}

// ===================== AIRFLOW CONTROL =====================

void setAirflowLevel(int index) {

  if (index < 0 || index > 2) {
    return;
  }

  currentAirIndex = index;

  airflowPosition = airflowLevels[index];

  airServo.write(airflowPosition);

  Serial.print("Airflow: ");

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

// ===================== CO2 SENSOR READING =====================

int readCO2(HardwareSerial &sensor) {

  while (sensor.available()) {
    sensor.read();
  }

  sensor.write( readCO2Command, sizeof(readCO2Command));

  byte response[9];

  if (sensor.readBytes(response, 9) != 9) {
    return -1;
  }

  if (response[0] != 0xFF ||
      response[1] != 0x86) {
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

  return (int)response[2] * 256 + response[3];
}

// ===================== TAKE ONE READING =====================

bool takeReading(float &difference) {

  updateFan();

  float lux = lightMeter.readLightLevel();

  int inlet = readCO2(CO2Sensor1);

  delay(200);

  int outlet = readCO2(CO2Sensor2);

  Serial.print("t=");
  Serial.print(millis() / 1000);

  Serial.print("s | T=");
  Serial.print(lastTempC);

  Serial.print("C | fan=");
  Serial.print(actualFanOn ? "ON" : "OFF");

  Serial.print(" | lux=");
  Serial.print(lux);

  Serial.print(" | PWM=");
  Serial.print(lightPWM);

  Serial.print(" | air=");

  if (currentAirIndex == 0) {
    Serial.print("HIGH");
  }
  else if (currentAirIndex == 1) {
    Serial.print("MED");
  }
  else {
    Serial.print("LOW");
  }

  if (inlet == -1 || outlet == -1) {

    Serial.println(" | CO2 SENSOR ERROR: SKIPPED");

  return false;
  }

  int correctedInlet = inlet - CO2_1_OFFSET;

  difference = (float)(correctedInlet - outlet);

  differenceHistory[historyPosition] = difference;

  historyPosition = (historyPosition + 1) % 18;

  if (historyCount < 18) {
    historyCount++;
  }

  float sum = 0;

  for (int i = 0; i < historyCount; i++) {
    sum += differenceHistory[i];
  }

  Serial.print(" | CO2 IN=");
  Serial.print(correctedInlet);

  Serial.print(" | OUT=");
  Serial.print(outlet);

  Serial.print(" | delta=");
  Serial.print(difference);

  Serial.print(" ppm | rolling=");
  Serial.print(sum / historyCount);

  Serial.println(" ppm");

  return true;
}

// ===================== MONITOR SYSTEM =====================

void monitorFor(unsigned long durationMs) {

  unsigned long start = millis();

  while (millis() - start < durationMs) {

    float difference = 0;

    takeReading(difference);

    delay(SAMPLE_MS);
  }
}

// ===================== MEASURE CO2 PERFORMANCE =====================

float measureCO2Score(unsigned long durationMs) {

  unsigned long start = millis();

  float total = 0;
  int count = 0;

  while (millis() - start < durationMs) {

    float difference = 0;

    if (takeReading(difference)) {

      total += difference;
      count++;
    }

    delay(SAMPLE_MS);
  }

  if (count == 0) {

    Serial.println("NO VALID CO2 DATA: TEST INVALID");

    return NAN;
  }

  float mean = total / count;

  Serial.print("Mean CO2 difference (ppm): ");
  Serial.print(mean);

  Serial.print(" | valid readings: ");
  Serial.println(count);

  return mean;
}

// ===================== SETTLE AND MEASURE =====================

float testCurrentSettings(
  unsigned long settleMs,
  unsigned long measureMs
) {

  Serial.println("Settling...");

  monitorFor(settleMs);

  Serial.println("Measuring...");

  return measureCO2Score(measureMs);
}

// ===================== LIGHT OPTIMIZATION =====================

void optimizeLightLevels() {

  Serial.println();
  Serial.println("===== LIGHT OPTIMIZATION =====");

  int startingIndex = currentLightIndex;

  float best = testCurrentSettings(
    LIGHT_AIR_SETTLE_MS,
    LIGHT_AIR_MEASURE_MS
  );

  if (isnan(best)) {

    Serial.println("Light baseline failed; unchanged.");

    return;
  }

  int bestIndex = startingIndex;

  for (int i = 0; i < 3; i++) {

    if (i == startingIndex) {
      continue;
    }

    setLightLevel(i);

    float score = testCurrentSettings(
      LIGHT_AIR_SETTLE_MS,
      LIGHT_AIR_MEASURE_MS
    );

    if (!isnan(score) &&
      score > best + IMPROVEMENT_MARGIN_PPM) {

      best = score;
      bestIndex = i;
    }
  }

  setLightLevel(bestIndex);

  Serial.print("Selected light PWM: ");
  Serial.println(lightPWM);
}

// ===================== AIRFLOW EXPERIMENT =====================

void optimizeAirflowLevels() {

  Serial.println();
  Serial.println("===== AIRFLOW EXPERIMENT =====");

  int startingIndex = currentAirIndex;

  bool calibrated = AIRFLOW_CALIBRATED;

  for (int i = 0; i < 3; i++) {

    if (!(airflowLpm[i] > 0.0f)) {
      calibrated = false;
    }
  }

  if (!calibrated) {

    Serial.println(
      "Uncalibrated: LOGGING ONLY."
    );

    Serial.println(
      "No airflow winner will be selected."
    );
  }

  float bestFlowScore = NAN;

  int bestIndex = startingIndex;

  for (int step = 0; step < 3; step++) {

    int i = (startingIndex + step) % 3;

    setAirflowLevel(i);

    float delta = testCurrentSettings(
      LIGHT_AIR_SETTLE_MS,
      LIGHT_AIR_MEASURE_MS
    );

    Serial.print("Airflow ");

    if (i == 0) {
      Serial.print("HIGH");
    }
    else if (i == 1) {
    Serial.print("MEDIUM");
    }
    else {
      Serial.print("LOW");
    }

    Serial.print(" | delta ppm: ");
    Serial.println(delta);

    if (calibrated && !isnan(delta)) {

    float flowScore = airflowLpm[i] * delta;

      Serial.print("Flow-adjusted score: ");
      Serial.println(flowScore);

    float threshold =
        IMPROVEMENT_MARGIN_PPM *
        fmaxf(airflowLpm[i], airflowLpm[bestIndex]);

      if (isnan(bestFlowScore) ||
          flowScore > bestFlowScore + threshold) {

        bestFlowScore = flowScore;
        bestIndex = i;
      }
    }
  }

  if (calibrated && !isnan(bestFlowScore)) {
    setAirflowLevel(bestIndex);
  }
  else {
    setAirflowLevel(startingIndex);
  }

  if (calibrated) {
    Serial.println("Airflow selection finished.");
  }
  else {
    Serial.println(
      "Restored original airflow; calibration needed."
  );
  }
}

// ===================== FAN EXPERIMENT =====================

void optimizeFanModes() {

  Serial.println();
  Serial.println("===== FAN EXPERIMENT =====");

  updateFan();

  if (isnan(lastTempC) ||
      lastTempC == DEVICE_DISCONNECTED_C ||
      thermalOverride ||
      lastTempC > MAX_TEMP_TO_START_FAN_TEST_C) {

    Serial.println(
      "Fan test skipped: water too warm or sensor unavailable.");

      return;
  }

bool oldFanTestMode = fanTestMode;

  bool oldRequestedFanOn = requestFanOn;

  safetyTriggeredDuringTrial = false;

  setFanMode(true);

  float onScore = testCurrentSettings(
    FAN_SETTLE_MS,
    FAN_MEASURE_MS
  );

  bool onWasOverridden = safetyTriggeredDuringTrial;

  safetyTriggeredDuringTrial = false;

  setFanMode(false);

  float offScore = testCurrentSettings(
    FAN_SETTLE_MS,
    FAN_MEASURE_MS
  );

  bool offWasOverridden = safetyTriggeredDuringTrial;

  Serial.print("Fan ON score: ");
  Serial.println(onScore);

  Serial.print("Fan OFF score: ");
  Serial.println(offScore);

  if (offWasOverridden || thermalOverride) {

    Serial.println( "OFF trial invalid / too warm: FAN ON for safety.");

    setFanMode(true);
  }
  else if (
    isnan(onScore) ||
    isnan(offScore) ||
    onWasOverridden ||
    fabsf(onScore - offScore) <= IMPROVEMENT_MARGIN_PPM
  ) {

    Serial.println("Inconclusive: restoring previous fan control mode.");

    fanTestMode = oldFanTestMode;
    requestFanOn = oldRequestedFanOn;

    updateFan();
  }
  else {

    setFanMode(onScore > offScore);

    Serial.println(
      "Fan mode chosen based on measured CO2 difference.");
  }
}

// ===================== CONTINUOUS OPTIMIZER =====================

void runOptimizationCycle() {

  Serial.println();
  Serial.println("========== OPTIMIZATION CYCLE START ==========");

  optimizeLightLevels();

  optimizeAirflowLevels();

  optimizeFanModes();

  Serial.println("========== OPTIMIZATION CYCLE COMPLETE ==========");
}

// ===================== SETUP =====================

void setup() {

  Serial.begin(115200);

  // LED PWM
  ledcAttach(LIGHT_PIN, 1200, 8);
  ledcWrite(LIGHT_PIN, lightPWM);

  // Temperature
  sensors.begin();

  // Fan
  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW);

  // Light sensor
  Wire.begin(21, 22);

  if (lightMeter.begin()) {
    Serial.println("BH1750 connected");
  }
  else {
    Serial.println("BH1750 NOT FOUND");
  }

  // Airflow servo
  airServo.setPeriodHertz(50);
  airServo.attach(SERVO_PIN, 500, 2400);
  airServo.write(airflowPosition);

  // CO2 sensors
  CO2Sensor1.begin(
    9600,
    SERIAL_8N1,
    CO2_1_RX,
    CO2_1_TX
  );

  CO2Sensor2.begin(
    9600,
    SERIAL_8N1,
    CO2_2_RX,
    CO2_2_TX
  );

  CO2Sensor1.setTimeout(1500);
  CO2Sensor2.setTimeout(1500);

  updateFan();

  if (RUN_OPTIMIZATION) {
    Serial.println("MODE: OPTIMIZATION");
  }
  else {
    Serial.println("MODE: CONTROL");
  }

  Serial.println("System started.");

  Serial.println("Allow CO2 sensors to warm up before interpreting results.");
}

// ===================== MAIN LOOP =====================

void loop() {

  if (RUN_OPTIMIZATION) {

    runOptimizationCycle();

    monitorFor(CYCLE_PAUSE_MS);
  }
  else {

    float difference = 0;

    takeReading(difference);

    delay(SAMPLE_MS);
  }
}
