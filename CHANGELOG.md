# Changelog

All notable changes to this project will be documented in this file.

## [1.0.0] - 2025-12-09

### ✨ New Features
- **Auto-Calibration Algorithm:** Implemented a warm-up logic that detects sensor resistance stability to automatically calibrate the `Ro` value outdoors.
- **Atmospheric CO2 Reference:** Configured base value of 427.48 ppm (Ref: Dec 2025).
- **Air Quality Indicator:** Added visual classification on LCD based on PPM ranges:
  - Excellent (< 800)
  - Good (< 1200)
  - Ventilate (< 2000)
  - Unhealthy (< 5000)
  - Toxic (> 5000)
- **LCD Interface:** Optimized layout to display PPM (line 1) and Status (line 2).

### 🐛 Fixes
- Resolved macro conflict with `LiquidCrystal_I2C` library. Renamed local variable `Rs` to `resistanceRs`.
- Added analog reading averaging (20 samples) to reduce electrical noise in measurements.

### 🔧 Technical
- Complete rewrite of the codebase to improve readability and maintainability.
- Parameterization of constants (Pin A0, MQ-135 Curve, Stability timings) for easy adjustment.
