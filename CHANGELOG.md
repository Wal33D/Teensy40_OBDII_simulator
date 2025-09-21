# Changelog

All notable changes to the Teensy40 OBDII Simulator project will be documented in this file.

## [1.2.0] - 2025-01-20

### Added
- Full OBD Mode 09 (Vehicle Information) support
  - PID 0x02: Vehicle Identification Number (VIN)
  - PID 0x04: Calibration ID
  - PID 0x06: Calibration Verification Number (CVN)
  - PID 0x0A: ECU Name
  - PID 0x0D: In-Use Performance Tracking
- ISO-TP multi-frame message handling for long responses
- Proper flow control implementation
- Realistic vehicle data simulation

### Improved
- Enhanced message buffer handling
- Better CAN frame construction
- More robust response timing

### Fixed
- Multi-frame message segmentation
- Flow control frame processing
- Response data formatting

## [1.1.0] - Previous Release

### Added
- Basic OBD-II Mode 01 support
- Live data simulation
- DTC (Diagnostic Trouble Code) support

### Initial Features
- CAN bus communication at 500kbps
- Standard OBD-II PID responses
- Teensy 4.0 platform support