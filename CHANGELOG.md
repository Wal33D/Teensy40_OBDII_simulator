# Changelog

## Changes from Original Fork

### Original Implementation (skpang.co.uk)
The original simulator had:
- **Mode 01**: Only 8 PIDs (RPM, Speed, Coolant, MAF, O2, Throttle, PID Support, Monitor Status)
- **Mode 03**: Request trouble codes
- **Mode 04**: Clear trouble codes and MIL
- Potentiometer-based value reading (static values)
- Basic CAN communication
- ~200 lines of code

### Our Enhancements

#### Mode 01 - Complete Overhaul
**Original**: 8 basic PIDs
**Now**: 44 PIDs with real Mercedes-Benz data

Added 36 new PIDs:
- All fuel trim PIDs (0x06-0x09)
- Intake pressure (0x0B)
- Timing advance (0x0E)
- Intake air temp (0x0F)
- O2 sensors present (0x13)
- Extended O2 sensors (0x15, 0x19)
- OBD standards (0x1C)
- Engine run time (0x1F)
- PIDs 0x20-0x40 support
- Distance with MIL (0x21)
- Fuel rail pressure (0x23)
- EVAP system (0x2E, 0x32)
- Fuel level (0x2F)
- Warm-ups (0x30)
- Distance since cleared (0x31)
- Barometric pressure (0x33)
- Advanced O2 sensors (0x34, 0x38)
- Catalyst temperatures (0x3C, 0x3D)
- PIDs 0x40-0x60 support
- Monitor status cycle (0x41)
- Battery voltage (0x42)
- Absolute load (0x43)
- Commanded equiv ratio (0x44)
- Relative throttle (0x45)
- Ambient air temp (0x46)
- Throttle position B (0x47)
- Accelerator positions (0x49, 0x4A)
- Commanded throttle (0x4C)
- Fuel type (0x51)
- O2 trim banks (0x56, 0x58)

**Replaced static potentiometer reading with dynamic simulation:**
- 5 driving states (IDLE, CITY, ACCELERATING, HIGHWAY, BRAKING)
- Automatic state transitions every 10 seconds
- Realistic value correlations (RPM/Speed/Load/Throttle)
- Smooth value updates every 100ms
- Dynamic MAF that scales with RPM
- Oscillating O2 sensors (0.35-0.45V)

#### Mode 02 - Freeze Frame (NEW)
**Original**: Not implemented
**Now**: Complete freeze frame support
- Captures sensor values when DTC triggered
- Stores 2 freeze frames
- Responds to all Mode 02 PID requests
- Proper frame number handling

#### Mode 09 - Vehicle Information (NEW)
**Original**: Not implemented
**Now**: Full vehicle information mode
- PID 0x00: Supported PIDs (multi-ECU simulation)
- PID 0x02: VIN (4JGDA5HB7JB158144)
- PID 0x04: Calibration ID (2769011200190170)
- PID 0x06: CVN (EB854939)
- PID 0x08: In-Use Performance Tracking
- PID 0x0A: ECU Name (ECM-EngineControl)
- PID 0x14: Auxiliary I/O Status
- ISO-TP multi-frame protocol implementation
- Flow control handling for long messages

### Technical Improvements
- **Code size**: From 199 lines to 1,085 lines (5.4x growth)
- **Switch cases**: From 8 to 89 (81 new cases)
- **Header additions**: 29 new PID definitions
- **Protocol compliance**: Perfect ISO 15765-4 CAN format
- **Response format**: Proper 41[PID][DATA] structure
- **Data accuracy**: Based on real Mercedes-Benz captures

### Data Source
All values based on real vehicle data captured from:
- **Vehicle**: Mercedes-Benz
- **VIN**: 4JGDA5HB7JB158144
- **Driving conditions**: Idle, City, Highway, Braking
- **Data points**: 7,510+ logged responses

---
**Modified by**: Waleed Judah (Wal33D)
**Email**: aquataze@yahoo.com
**Original by**: skpang.co.uk