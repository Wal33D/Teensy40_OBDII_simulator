# Teensy40 OBDII Simulator - Enhanced Edition

Advanced OBD-II ECU simulator for Teensy 4.0 with CAN-BUS support, enhanced with Mode 09 Vehicle Information capabilities.

## Original Features (skpang)

The original simulator by [skpang](https://www.skpang.co.uk) (December 2022) included:
- Mode 01: Show current data (Live Data PIDs)
- Mode 03: Show stored Diagnostic Trouble Codes
- Mode 04: Clear Diagnostic Trouble Codes

## Enhancements by Wal33D (2025)

### Mode 09 - Vehicle Information Implementation
Complete implementation of OBD-II Mode 09 for vehicle identification and tracking:

#### Implemented PIDs:
- **PID 0x02** - Vehicle Identification Number (VIN)
  - 17-character VIN support
  - Multi-frame ISO-TP message handling

- **PID 0x04** - Calibration ID
  - Multiple calibration ID support
  - Dynamic length handling

- **PID 0x06** - Calibration Verification Number (CVN)
  - CVN calculation and reporting
  - Multiple CVN support

- **PID 0x0A** - ECU Name
  - 20-character ECU name reporting
  - ASCII string transmission

- **PID 0x0D** - In-Use Performance Tracking
  - Spark ignition monitors
  - Compression ignition monitors

### Technical Improvements
- **ISO-TP Protocol**: Full implementation of ISO 15765-2 multi-frame messaging
- **Flow Control**: Proper flow control frame handling for long messages
- **Buffer Management**: Enhanced buffer handling for variable-length responses
- **Message Segmentation**: Automatic segmentation of long responses into CAN frames

## Hardware Requirements

Compatible with:
- [Teensy 4.0 OBDII CAN-BUS ECU Simulator](https://www.skpang.co.uk/collections/teensy/products/teensy-4-0-obdii-can-bus-ecu-simulator-with-teensy-4-0)
- 500kbps CAN communication
- Standard OBD-II connector

## Installation

1. Install [Teensyduino](https://www.pjrc.com/teensy/teensyduino.html)
2. Clone this repository
3. Open `Teensy40_OBDII_simulator.ino` in Arduino IDE
4. Select Tools → Board → Teensy 4.0
5. Compile and upload to Teensy

## Usage

The simulator responds to standard OBD-II requests on CAN ID 0x7DF and replies on 0x7E8.

### Example Mode 09 Request
```
Request VIN: 7DF 02 09 02
Response:    7E8 10 14 49 02 01 57 41 55
            7E8 21 5A 5A 5A 41 46 33 38
            7E8 22 5A 54 41 31 32 33 34
```

## Testing

Use any OBD-II scanner or diagnostic tool that supports Mode 09 to verify functionality.

## Changelog

See [CHANGELOG.md](CHANGELOG.md) for detailed version history.

## Contributing

Fork this repository and submit pull requests for any enhancements.

## Credits

- **Original Design**: [skpang](https://www.skpang.co.uk) (December 2022)
- **Mode 09 Implementation**: [Wal33D](https://github.com/Wal33D) (January 2025)

## License

See [LICENSE](LICENSE) file for details.

## References

- [OBD-II PIDs Wikipedia](http://en.wikipedia.org/wiki/OBD-II_PIDs)
- [ISO 15765-2](https://en.wikipedia.org/wiki/ISO_15765-2) - Transport protocol