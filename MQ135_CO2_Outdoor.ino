/*
 * MQ-135 CO2 Sensor - Outdoor Calibration
 *
 * Logic:
 * 0. Thermal Acclimatization: IF moving from warm indoors to cold outdoors,
 *    leave the device OFF for 10-15 mins outside to let the sensor body cool
 * down. Otherwise, cooling during calibration will cause Rs drift and wrong low
 * PPM values.
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
const float RL_VALUE =
    1.0; // Load Resistor on board in KOhms.
         // IMPORTANTE: Revisar el modulo. Muchos tienen 1k (1.0) o 10k (10.0).
         // Si los valores son muy altos, prueba cambiar a 10.0

const float ATM_CO2 = 427.48; // CO2 value (Dec 7 2025)

// MQ-135 Curve for CO2 (approximate parameters)
// Curve format: ppm = A * (Rs/Ro)^B
const float PARA_A = 116.602;
const float PARA_B = -2.769;

// Stability settings
// Stability settings
const int STABLE_CYCLE_MS = 1000; // Match Loop speed (1s) for power consistency
const float STABILITY_THRESHOLD = 0.5; // Max deviation in % to consider stable
const int STABILITY_SAMPLES = 60;      // Increase samples (60 * 1s = 1 min)

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
  // ... (skip unchanged lines) ...
  delay(1000);

  // Calibration Routine
  calibrateSensor();
}

void loop() {
  float currentRs = readRs();

  // EMA Filter
  if (smoothedRs == 0)
    smoothedRs = currentRs;
  smoothedRs = 0.1 * currentRs + 0.9 * smoothedRs;

  float ppm = getPPM(smoothedRs, Ro);

  // Output to Serial (Unchanged)
  // ...
}

void calibrateSensor() {
  lcd.clear();
  lcd.print("Warming up...");
  Serial.println("Waiting for sensor stability...");

  float lastRs = 0;
  int stableCount = 0;
  unsigned long timer = millis();

  // Initialize filter
  smoothedRs = readRs();

  // Wait loop until stability is reached
  while (stableCount < STABILITY_SAMPLES) {
    if (millis() - timer > STABLE_CYCLE_MS) {
      timer = millis();
      float currentRs = readRs();

      // Update EMA during calibration
      smoothedRs = 0.1 * currentRs + 0.9 * smoothedRs;

      // Check stability on SMOOTHED value to avoid noise resetting counter
      // Or check RAW if strict. Let's check MEAN (Smoothed) variation
      // but compared to previous smoothed.

      float deviation = 100.0;
      if (lastRs > 0) {
        deviation = abs(smoothedRs - lastRs) / lastRs * 100.0;
      }

      Serial.print("Rs(sm): ");
      Serial.print(smoothedRs);
      Serial.print(" Dev: ");
      Serial.print(deviation);
      Serial.println("%");

      // Update LCD Status - mimics loop load
      lcd.setCursor(0, 1);
      lcd.print("Rs:");
      lcd.print(smoothedRs, 1);
      lcd.print("k ");

      lcd.setCursor(14, 1);
      lcd.print(stableCount % 2 == 0 ? "." : "o");

      if (deviation < STABILITY_THRESHOLD && lastRs > 0) {
        stableCount++;
      } else {
        // If deviating, reset, but maybe don't reset fully if just minor noise?
        // Keep strict reset for now.
        if (deviation > STABILITY_THRESHOLD)
          stableCount = 0;
      }

      lastRs = smoothedRs; // Compare against smoothed
    }
  }

  // Stability reached, calculate Ro using SMOOTHED Rs
  Serial.println("Stable! Calibrating to 427.48 ppm...");
  Serial.flush();

  lcd.clear();
  lcd.print("Calib to 427ppm");
  delay(1000);

  // Correction: If current PPM is ATM_CO2
  // Factor = (ATM_CO2 / PARA_A) ^ (1 / PARA_B)
  // Factor = (427.48 / 116.602) ^ (1 / -2.769) approx 0.623
  // Calculating manually to avoid runtime pow issues if any.
  float base = ATM_CO2 / PARA_A;
  float exponent = 1.0 / PARA_B;
  float factor = pow(base, exponent);

  Serial.print("Factor: ");
  Serial.println(factor);

  if (factor == 0)
    factor = 0.623; // Safety fallback

  Ro = smoothedRs / factor; // Use smoothedRs here

  Serial.print("Calibrated Ro: ");
  Serial.println(Ro);

  lcd.setCursor(0, 1);
  lcd.print("Done! Ro:");
  lcd.print(Ro);
  delay(2000);
}

float readRs() {
  // Read analog and convert to resistance
  // Circuit: VCC -> MQ135 -> A0 -> RL -> GND
  // Vout = VCC * (RL / (Rs + RL)) => Rs = RL * (VCC/Vout - 1)
  // ADC = (Vout/VCC)*1023 => VCC/Vout = 1023/ADC
  // Rs = RL * (1023/ADC - 1)

  long adc_sum = 0;
  for (int i = 0; i < 20; i++) { // Average 20 readings for better stability
    adc_sum += analogRead(PIN_MQ135);
    delay(10);
  }
  float adc = adc_sum / 20.0;

  // Prevent division by zero or negative
  if (adc < 1)
    adc = 1;
  if (adc >= 1023)
    adc = 1022;

  float rs = RL_VALUE * (1023.0 / adc - 1.0);
  return rs;
}

float getPPM(float rs, float ro) {
  if (ro == 0)
    return 0; // Safety
  float ratio = rs / ro;
  double ppm = PARA_A * pow(ratio, PARA_B);
  return (float)ppm;
}
