# Teensy 4.0 OBD-II CAN Bus ECU Simulator

An advanced OBD-II ECU simulator for Teensy 4.0 with comprehensive vehicle diagnostics support, dynamic driving simulation, and real Mercedes-Benz data.

## Features

### Complete OBD-II Protocol Support

#### Mode 01 - Real-Time Data (44 PIDs)
- **Engine Performance**: RPM, speed, coolant temperature, intake temperature
- **Fuel System**: Fuel trims (STFT/LTFT), fuel level, fuel type, fuel rail pressure
- **Emission Controls**: O2 sensors, catalyst temperatures, EVAP system
- **Throttle/Load**: Throttle positions, absolute load, commanded throttle
- **Environmental**: Ambient temperature, barometric pressure
- **Diagnostic**: Monitor status, distance with MIL, warm-ups since cleared

#### Mode 02 - Freeze Frame Data
- Captures and stores sensor values when DTCs are triggered
- Support for 2 independent freeze frames
- Full PID response for frozen data

#### Mode 03 - Diagnostic Trouble Codes
- Returns stored trouble codes (P0118, P0130)
- Proper DTC formatting

#### Mode 04 - Clear DTCs
- Clears stored trouble codes and MIL light
- Resets freeze frame data

#### Mode 09 - Vehicle Information
- **VIN**: 4JGDA5HB7JB158144 (Mercedes-Benz)
- **Calibration ID**: 2769011200190170
- **CVN**: EB854939
- **ECU Name**: ECM-EngineControl
- **Multi-frame ISO-TP support** for long messages
- **In-Use Performance Tracking**

### Dynamic Driving Simulation

The simulator cycles through realistic driving states:

1. **IDLE** (800 RPM, 0 km/h)
   - Engine idling conditions
   - Low load, minimal throttle

2. **CITY** (1800 RPM, 45 km/h)
   - Urban driving simulation
   - Moderate load and throttle

3. **ACCELERATING** (3500 RPM, 90 km/h)
   - Highway acceleration
   - High load, full throttle

4. **HIGHWAY** (2500 RPM, 120 km/h)
   - Steady cruising speed
   - Moderate load, partial throttle

5. **BRAKING** (1200 RPM, 30 km/h)
   - Deceleration conditions
   - Low load, minimal throttle

**Features:**
- Automatic state transitions every 10 seconds
- Smooth value interpolation between states
- Correlated engine parameters (RPM/Speed/Load/MAF)
- Dynamic O2 sensor oscillation (0.35-0.45V)
- Updates every 100ms for realistic behavior

## Hardware Requirements

- **Teensy 4.0** microcontroller
- **CAN Bus transceiver** (MCP2562 or similar)
- **OBD-II connector** wiring
- Optional: Status LEDs (red/green)

### Pin Connections
```
Teensy 4.0    ->  CAN Transceiver
Pin 22 (CTX1) ->  TXD
Pin 23 (CRX1) ->  RXD
GND           ->  GND
3.3V          ->  VCC

Status LEDs:
Pin 13        ->  Built-in LED
Pin 19        ->  Red LED (optional)
Pin 20        ->  Green LED (optional)
```

## Installation

1. Install [Teensyduino](https://www.pjrc.com/teensy/teensyduino.html) for Arduino IDE
2. Install required library:
   - FlexCAN_T4 (via Library Manager)
3. Clone this repository:
   ```bash
   git clone https://github.com/Wal33D/Teensy40_OBDII_simulator.git
   ```
4. Open `Teensy40_OBDII_simulator.ino` in Arduino IDE
5. Select **Tools > Board > Teensy 4.0**
6. Select **Tools > USB Type > Serial**
7. Upload to Teensy 4.0

## Usage

1. Connect the Teensy to your vehicle's OBD-II port or diagnostic tool
2. Power on the device (via USB or OBD-II connector)
3. The simulator automatically starts broadcasting responses
4. LEDs indicate status:
   - **Green flash**: Normal operation
   - **Red flash**: CAN message received
   - **Built-in LED**: Heartbeat (500ms interval)

## Testing

Compatible with any OBD-II scanner or software:
- **Torque Pro** (Android)
- **OBD Fusion** (iOS/Android)
- **ScanTool.net OBDLink**
- **ELM327** based tools
- Custom diagnostic software

## Data Accuracy

All simulated values are based on real vehicle data:
- **Source Vehicle**: Mercedes-Benz
- **Data Points**: 7,510+ logged OBD responses
- **Driving Conditions**: Real-world idle, city, highway, and braking scenarios
- **Protocol Compliance**: ISO 15765-4 CAN (11-bit, 500kbps)

## Code Statistics

### Evolution from Original
- **Original Code**: 199 lines (basic 8 PIDs)
- **Enhanced Code**: 1,085+ lines (44 PIDs + modes)
- **Growth Factor**: 5.4x
- **New Features**: 81 additional switch cases
- **Protocol Support**: Added ISO-TP multi-frame handling

### File Structure
```
Teensy40_OBDII_simulator/
├── Teensy40_OBDII_simulator.ino  # Main sketch
├── ecu_sim.cpp                    # ECU simulation implementation
├── ecu_sim.h                      # Headers and definitions
├── CHANGELOG.md                   # Detailed change history
└── README.md                      # This file
```

## Version History

### v2.0.0 (2024) - Major Enhancement by Wal33D
- Added 36 new Mode 01 PIDs
- Implemented Mode 02 (Freeze Frame)
- Implemented Mode 09 (Vehicle Information)
- Dynamic driving state simulation
- ISO-TP multi-frame protocol support
- Real Mercedes-Benz data integration

### v1.0.0 (2022) - Original by skpang.co.uk
- Basic Mode 01 (8 PIDs)
- Mode 03/04 DTC support
- Static potentiometer-based values

## Technical Details

### CAN Configuration
- **Baud Rate**: 500kbps (standard OBD-II)
- **CAN ID Request**: 0x7DF (broadcast) or 0x7E0 (ECU specific)
- **CAN ID Response**: 0x7E8
- **Protocol**: ISO 15765-4

### Response Format
```
Standard Response: [Mode+0x40] [PID] [Data...]
Example Mode 01: 41 0C XX XX (RPM response)
Example Mode 09: 49 02 01 [VIN data...] (VIN response)
```

### Multi-Frame Messages (ISO-TP)
For responses > 8 bytes (like VIN):
1. First Frame: `10 [length] [data...]`
2. Flow Control: Wait for `30 00 00`
3. Consecutive Frames: `21 [data...]`, `22 [data...]`

## Contributing

Contributions are welcome! Please ensure:
1. Code follows existing style
2. Comments explain complex logic
3. Test with real OBD-II scanners
4. Update CHANGELOG.md

## License

This project is based on original work by skpang.co.uk (2022) with significant enhancements.

## Author

**Enhanced Version (v2.0.0)**
- Waleed Judah (Wal33D)
- Email: aquataze@yahoo.com
- GitHub: [@Wal33D](https://github.com/Wal33D)

**Original Version (v1.0.0)**
- skpang.co.uk
- [Product Page](https://www.skpang.co.uk/collections/teensy/products/teensy-4-0-obdii-can-bus-ecu-simulator-with-teensy-4-0)

## Acknowledgments

- Original simulator framework by skpang.co.uk
- FlexCAN_T4 library by tonton81
- Real vehicle data from Mercedes-Benz diagnostics
- OBD-II PID documentation from Wikipedia

---

**Note**: This simulator is for educational and development purposes. Always comply with local regulations when working with vehicle systems.