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

const float ATM_CO2 = 427.48; // Valor CO2 diario (Dec 7 2025)

// MQ-135 Curve for CO2 (approximate parameters)
// Curve format: ppm = A * (Rs/Ro)^B
const float PARA_A = 116.602;
const float PARA_B = -2.769;

// Stability settings
const int STABLE_CYCLE_MS = 2000;      // Check stability every 2 seconds
const float STABILITY_THRESHOLD = 0.5; // Max deviation in % to consider stable
const int STABILITY_SAMPLES =
    30; // Number of consecutive stable checks needed (approx 1 min stable)

// Objects
// Ajustar dirección 0x27 o 0x3F según el módulo I2C
LiquidCrystal_I2C lcd(0x27, 16, 2);

float Ro = 10.0; // Will be calibrated
bool calibrated = false;

float readRs();
float getPPM(float rs, float ro);
void calibrateSensor();

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();

  pinMode(PIN_MQ135, INPUT);

  lcd.setCursor(0, 0);
  lcd.print("MQ-135 Init...");
  delay(2000);

  // Calibration Routine
  calibrateSensor();
}

void loop() {
  float resistanceRs = readRs();
  float ppm = getPPM(resistanceRs, Ro);

  // Output to Serial
  Serial.print("Rs: ");
  Serial.print(resistanceRs);
  Serial.print(" kOhm | CO2: ");
  Serial.print(ppm);
  Serial.println(" ppm");

  // Determine Quality
  String status = "";
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
  lcd.print(ppm, 0); // No decimals needed for high values
  lcd.print(" ppm");

  lcd.setCursor(0, 1);
  lcd.print(status);

  delay(1000); // 1 second update rate
}

void calibrateSensor() {
  lcd.clear();
  lcd.print("Warming up...");
  Serial.println("Waiting for sensor stability...");

  float lastRs = 0;
  int stableCount = 0;
  unsigned long timer = millis();

  // Wait loop until stability is reached
  while (stableCount < STABILITY_SAMPLES) {
    if (millis() - timer > STABLE_CYCLE_MS) {
      timer = millis();
      float currentRs = readRs();

      // Calculate deviation
      float deviation = 100.0;
      if (lastRs > 0) {
        deviation = abs(currentRs - lastRs) / lastRs * 100.0;
      }

      Serial.print("Rs: ");
      Serial.print(currentRs);
      Serial.print(" Dev: ");
      Serial.print(deviation);
      Serial.println("%");

      // Update LCD Status
      lcd.setCursor(0, 1);
      lcd.print("Rs:");
      lcd.print(currentRs, 1);
      lcd.print("k ");
      // Show progress bar or indicator
      lcd.setCursor(14, 1);
      lcd.print(stableCount % 2 == 0 ? "." : "o");

      if (deviation < STABILITY_THRESHOLD && lastRs > 0) {
        stableCount++;
      } else {
        stableCount = 0; // Reset if unstable
        // Optional: If it takes too long, maybe increase threshold?
      }

      lastRs = currentRs;
    }
  }

  // Stability reached, calculate Ro
  // Equation: ppm = A * (ratio)^B  => ppm = A * (Rs/Ro)^B
  // (ppm/A)^(1/B) = Rs/Ro
  // Ro = Rs / ((ppm/A)^(1/B))

  Serial.println("Stable! Calibrating...");
  lcd.clear();
  lcd.print("Stable! Calc Ro");
  delay(1000);

  // Correction: If current PPM is ATM_CO2
  float factor = pow(ATM_CO2 / PARA_A, 1.0 / PARA_B);
  Ro = lastRs / factor;

  Serial.print("Calibrated Ro: ");
  Serial.println(Ro);
  lcd.setCursor(0, 1);
  lcd.print("Ro: ");
  lcd.print(Ro);
  delay(3000);
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
