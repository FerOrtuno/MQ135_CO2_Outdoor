/*
 * MQ-135 CO2 Sensor - Outdoor Calibration
 *
 * Logic:
 * 0. Thermal Acclimatization: IF moving from warm indoors to cold outdoors,
 *    leave the device OFF for 10-15 mins outside to let the sensor body cool
 *    down. Otherwise, cooling during calibration will cause Rs drift and wrong
 * low PPM values.
 * 1. Warmup Phase: Sensor heats up. We monitor resistance stability.
 * 2. Calibration: Uses known Atmospheric CO2 (e.g. 427.48 ppm) to calibrate Ro.
 * 3. Measurement: Displays CO2 ppm based on calibrated Ro.
 *
 * Hardware:
 * - MQ-135 Analog Out connected to Pin A0
 * - LCD 16x2 I2C connected to SDA/SCL (A4/A5 on Uno/Nano)
 */

#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// --- CONFIGURATION ---
const int PIN_MQ135 = A0;
const float RL_VALUE = 1.0; // Load Resistor on board in KOhms.

const float ATM_CO2 = 427.48; // CO2 value (Dec 7 2025)

// MQ-135 Curve for CO2 (approximate parameters)
// Curve format: ppm = A * (Rs/Ro)^B
const float PARA_A = 116.602;
const float PARA_B = -2.769;

// Stability settings
const int STABLE_CYCLE_MS = 1000; // Match Loop speed (1s) for power consistency
const float STABILITY_THRESHOLD = 0.5; // Max deviation in % to consider stable
const int STABILITY_SAMPLES = 60;      // Increase samples (60 * 1s = 1 min)

// Empirical Compensation
// The sensor consistently drifts Rs upwards by ~5% after calibration phase
// (likely due to thermal changes when entering main loop), causing PPM to drop.
// Observed: Drops from 427ppm to ~378ppm. Correction: 427.48 / 378 ~= 1.13
const float COMPENSATION_FACTOR = 1.13;

// Objects
// Ajustar dirección 0x27 o 0x3F según el módulo I2C
LiquidCrystal_I2C lcd(0x27, 16, 2);

float Ro = 10.0; // Will be calibrated
bool calibrated = false;
float smoothedRs = 0; // Global EMA filter

float readRs();
float getPPM(float rs, float ro);
void calibrateSensor();

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();

  pinMode(PIN_MQ135, INPUT);

  // Initial Warning Sequence
  lcd.clear();
  lcd.print("IMPORTANT:");
  lcd.setCursor(0, 1);
  lcd.print("ACCLIMATIZATION");
  delay(2000);

  lcd.clear();
  lcd.print("move Outdoor");
  lcd.setCursor(0, 1);
  lcd.print("Wait 10min OFF");
  delay(3000);

  lcd.clear();
  lcd.print("MQ-135 Init...");

  Serial.println("--------------------------------------------------");
  Serial.println("IMPORTANT: If moving from indoors to outdoors,");
  Serial.println("leave device OFF for 10 mins outside before ON");
  Serial.println("to allow thermal acclimation of the sensor body.");
  Serial.println("--------------------------------------------------");

  delay(1000);

  // Calibration Routine
  calibrateSensor();
  Serial.println("Setup done. Entering Loop...");
  Serial.flush();
}

void loop() {
  Serial.println("Loop start");
  float currentRs = readRs();

  // EMA Filter
  if (smoothedRs == 0)
    smoothedRs = currentRs;
  smoothedRs = 0.1 * currentRs + 0.9 * smoothedRs;

  float ppmBase = getPPM(smoothedRs, Ro);
  float ppm = ppmBase * COMPENSATION_FACTOR;

  // Output to Serial
  Serial.print("Rs(raw): ");
  Serial.print(currentRs);
  Serial.print(" | Rs(smooth): ");
  Serial.print(smoothedRs);
  Serial.print(" kOhm | CO2: ");
  Serial.print(ppmBase);
  Serial.print(" x ");
  Serial.print(COMPENSATION_FACTOR);
  Serial.print(" -> ");
  Serial.print(ppm);
  Serial.println(" ppm");

  // Determine Quality
  const char *status = "";
  if (ppm < 800) {
    status = "Air: Excellent";
  } else if (ppm < 1200) {
    status = "Air: Good";
  } else if (ppm < 2000) {
    status = "Warn: Ventilate";
  } else if (ppm < 5000) {
    status = "Unhealthy!";
  } else {
    status = "DANGER! TOXIC";
  }

  // Output to LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("CO2: ");
  lcd.print(ppm, 0);
  lcd.print(" ppm");

  lcd.setCursor(0, 1);
  lcd.print(status);

  delay(1000);
}

void calibrateSensor() {
  lcd.clear();
  lcd.print("Warming up...");
  Serial.println("Waiting for sensor stability...");

  float lastRs = 0;
  int stableCount = 0;
  unsigned long timer = millis();

  // PRE-HEATING: Mandatory 3 minutes
  const int PREHEAT_SECONDS = 180;
  for (int i = 0; i < PREHEAT_SECONDS; i++) {
    lcd.setCursor(0, 0);
    lcd.print("Pre-heating...  ");
    lcd.setCursor(0, 1);
    lcd.print("Wait: ");
    lcd.print(PREHEAT_SECONDS - i);
    lcd.print("s   ");

    readRs();

    Serial.print("Pre-heat: ");
    Serial.print(PREHEAT_SECONDS - i);
    Serial.println("s");
    delay(1000);
  }

  // Initialize filter with current value after pre-heat
  smoothedRs = readRs();

  // Wait loop until stability is reached
  while (stableCount < STABILITY_SAMPLES) {
    if (millis() - timer > STABLE_CYCLE_MS) {
      timer = millis();
      float currentRs = readRs();

      // Update EMA
      smoothedRs = 0.1 * currentRs + 0.9 * smoothedRs;

      float deviation = 100.0;
      if (lastRs > 0) {
        deviation = abs(smoothedRs - lastRs) / lastRs * 100.0;
      }

      Serial.print("Rs(sm): ");
      Serial.print(smoothedRs);
      Serial.print(" Dev: ");
      Serial.print(deviation);
      Serial.println("%");

      // Update LCD
      lcd.setCursor(0, 1);
      lcd.print("Rs:");
      lcd.print(smoothedRs, 1);
      lcd.print("k ");

      lcd.setCursor(14, 1);
      lcd.print(stableCount % 2 == 0 ? "." : "o");

      if (deviation < STABILITY_THRESHOLD && lastRs > 0) {
        stableCount++;
      } else {
        if (deviation > STABILITY_THRESHOLD)
          stableCount = 0;
      }

      lastRs = smoothedRs;
    }
  }

  // Stability reached
  Serial.println("Stable! Calibrating to 427.48 ppm...");
  Serial.flush();

  lcd.clear();
  lcd.print("Calib to 427ppm");
  delay(1000);

  // Correction
  float base = ATM_CO2 / PARA_A;
  float exponent = 1.0 / PARA_B;
  float factor = pow(base, exponent);

  Serial.print("Factor: ");
  Serial.println(factor);

  if (factor == 0)
    factor = 0.623;

  Ro = smoothedRs / factor;

  Serial.print("Calibrated Ro: ");
  Serial.println(Ro);

  lcd.setCursor(0, 1);
  lcd.print("Done! Ro:");
  lcd.print(Ro);
  delay(2000);
}

float readRs() {
  long adc_sum = 0;
  for (int i = 0; i < 20; i++) {
    adc_sum += analogRead(PIN_MQ135);
    delay(10);
  }
  float adc = adc_sum / 20.0;

  if (adc < 1)
    adc = 1;
  if (adc >= 1023)
    adc = 1022;

  float rs = RL_VALUE * (1023.0 / adc - 1.0);
  return rs;
}

float getPPM(float rs, float ro) {
  if (ro == 0)
    return 0;
  float ratio = rs / ro;
  double ppm = PARA_A * pow(ratio, PARA_B);
  return (float)ppm;
}
