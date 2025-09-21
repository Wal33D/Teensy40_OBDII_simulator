# Changelog

All notable changes to the Teensy40 OBDII Simulator project will be documented in this file.

## [2.0.0] - 2025-01-20
### Added by Wal33D
- **Mode 09 - Vehicle Information** - Complete implementation
  - PID 0x02: Vehicle Identification Number (VIN) - 17 characters
  - PID 0x04: Calibration ID - Multiple calibration IDs support
  - PID 0x06: Calibration Verification Number (CVN)
  - PID 0x0A: ECU Name - 20-character ECU identification
  - PID 0x0D: In-Use Performance Tracking monitors

### Technical Enhancements by Wal33D
- **ISO-TP Multi-Frame Protocol**: Full implementation for long message handling
- **Flow Control**: Proper flow control frame processing (0x30, 0x00, 0x00)
- **Dynamic Buffer Management**: Variable-length response handling
- **Message Segmentation**: Automatic CAN frame segmentation for responses > 7 bytes
- **Enhanced State Machine**: Robust message state tracking

### Fixed by Wal33D
- Multi-frame message assembly and transmission
- Response timing for diagnostic tools compatibility
- Buffer overflow protection for long messages

## [1.0.0] - 2022-12
### Original Implementation by skpang
- **Mode 01**: Show current data
  - Engine RPM
  - Vehicle Speed
  - Engine Coolant Temperature
  - Throttle Position
  - Other standard PIDs
- **Mode 03**: Show stored Diagnostic Trouble Codes
- **Mode 04**: Clear Diagnostic Trouble Codes and stored values
- CAN bus communication at 500kbps
- Teensy 4.0 platform support
- Basic OBD-II protocol compliance

## Notes

Version 2.0.0 represents a major enhancement focused on Mode 09 implementation, adding critical vehicle identification capabilities that were not present in the original simulator. All Mode 09 functionality and ISO-TP multi-frame handling was implemented by Wal33D.