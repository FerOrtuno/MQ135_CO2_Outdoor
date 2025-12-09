# MQ-135 CO2 Sensor - Outdoor Calibration

This project allows you to build an affordable and effective air quality and CO2 concentration detector using an MQ-135 sensor and an Arduino.

The system features an **outdoor auto-calibration** function, which adjusts the sensor's baseline to current atmospheric CO2 levels, ensuring much more accurate measurements than standard uncalibrated sketches.

## 📦 Required Materials

To assemble this project you will need:

*   **Microcontroller:** Arduino Nano V3 (recommended for size) or Arduino Uno.
    *   *Note:* If you use a clone with the CH340 chip, ensure you have the drivers installed.
*   **Gas Sensor:** MQ-135 Module.
*   **Display:** 16x2 LCD with I2C interface.
*   **Cables:** Jumper Wires (Female-Female or Male-Female depending on your board).
*   **Power:** USB Mini-B Cable (for Nano) or Type-B (for Uno).

## 🔌 Connections

Wiring is very simple:

| MQ-135 Pin | Arduino Pin |
| :--- | :--- |
| VCC | 5V |
| GND | GND |
| A0 (Analog) | **A0** |
| D0 (Digital) | *Not connected* |

| LCD I2C Pin | Arduino Pin (Nano/Uno) |
| :--- | :--- |
| VCC | 5V |
| GND | GND |
| SDA | **A4** |
| SCL | **A5** |

## ⚙️ Configuration and Upload

1.  **Install Arduino IDE:** Download it from the official website.
2.  **Install Libraries:**
    *   Open the IDE and go to *Tools > Manage Libraries*.
    *   Search for and install: `LiquidCrystal I2C` by *Frank de Brabander*.
3.  **Configure the Code:**
    *   Open `MQ135_CO2_Outdoor.ino`.
    *   Verify your display's I2C address in the line `LiquidCrystal_I2C lcd(0x27, 16, 2);`. If it doesn't work, try `0x3F`.
    *   Verify the `ATM_CO2` value if you wish to update the atmospheric reference (default 427.48 ppm).
4.  **Upload the Sketch:**
    *   Select the board `Arduino Nano`.
    *   **Important:** If using a Chinese clone, select Processor: `ATmega328P (Old Bootloader)`.

## 🚀 Usage and Calibration

1.  **Thermal Acclimatization (CRITICAL):**
    *   If there is a significant temperature difference (e.g., taking it from a 20°C room to 13°C outdoors), **leave the device OUTDOORS for 10-15 minutes BEFORE powering it on**.
    *   *Reason:* The sensor body needs to physically adjust to the ambient temperature. If calibrated while the sensor is still cooling down, the resistance will drift upwards, causing the calibration to result in incorrect, low PPM readings.

2.  **Power On:** Connect the power. You will see "MQ-135 Init...".

3.  **Warm-up and Calibration (Outdoors):**
    *   Ensure the sensor is outdoors (refreshing balcony, window, or garden).
    *   The system will enter "Warming up..." mode.
    *   It will wait for the sensor to heat up and the resistance reading to stabilize (approx. 5-10 minutes).
    *   Once stable, it automatically calculates the reference value (`Ro`) based on clean atmospheric CO2.

4.  **Measurement:**
    *   Once calibrated, the PPM value and air status will appear.
    *   You can now take it indoors to measure air quality.

## 📊 Result Interpretation

The display shows the value in PPM and a text indicator:

*   **< 800 ppm:** `Air: Excellent` (Fresh and ideal air)
*   **800 - 1200 ppm:** `Air: Good` (Normal indoor level)
*   **1200 - 2000 ppm:** `Warn: Ventilate` (Stuffy air, open windows)
*   **2000 - 5000 ppm:** `Unhealthy!` (Unhealthy, possible headaches)
*   **> 5000 ppm:** `DANGER! TOXIC` (Dangerous to health)

## ⚖️ License

This project is based on previous works from the Arduino community. Code rewritten and optimized for outdoor calibration.
Free for educational and personal use.
