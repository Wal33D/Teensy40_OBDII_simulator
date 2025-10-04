# OBD-II Mode 04 - Clear/Reset Emissions Diagnostic Information

## Overview

Mode 04 (Service ID: 0x04) is a critical OBD-II diagnostic command that performs a comprehensive reset of all emissions-related diagnostic data. This mode is mandated by the SAE J1979 standard and is essential for emissions testing, vehicle maintenance, and diagnostic workflows.

**Important Warning**: Clearing diagnostic codes does NOT fix the underlying mechanical or electrical problem. If the fault condition that triggered the code still exists, the Malfunction Indicator Lamp (MIL) will re-illuminate and Diagnostic Trouble Codes (DTCs) will be set again after the vehicle completes the next relevant drive cycle.

## Purpose in OBD-II

Mode 04 serves several critical purposes in the emissions monitoring system:

1. **Post-Repair Verification**: After repairs are completed, technicians use Mode 04 to clear codes and reset monitors. The vehicle must then complete drive cycles to verify the repair was successful.

2. **Emissions Testing Preparation**: Some jurisdictions require a full monitor readiness cycle before emissions testing. Mode 04 is used to reset all monitors.

3. **Diagnostic Workflow**: Allows technicians to clear intermittent codes and observe if they return, helping differentiate between persistent faults and one-time events.

4. **Compliance Reset**: Resets the emissions monitoring system to its initial state, allowing monitors to run fresh test cycles.

## What Gets Cleared

When Mode 04 is executed, the following data is comprehensively reset:

### 1. Diagnostic Trouble Codes (DTCs)
- **Stored DTCs**: All "matured" emissions-related DTCs that triggered the MIL are cleared
- **Pending DTCs**: Codes from the first detection (not yet matured) are also erased
- **DTC Count**: The number of stored DTCs is reset to zero
- **Freeze Frame Association**: Links between DTCs and their freeze frames are removed

### 2. Malfunction Indicator Lamp (MIL) Status
- **MIL Off**: The Check Engine Light is turned off immediately
- **MIL Status Flags**: All MIL-related status bits in Mode 01 PID 01 are cleared
- **Distance with MIL**: Counter tracking distance traveled with MIL illuminated is reset
- **Time with MIL**: Duration counters for MIL-on time are cleared

### 3. Freeze Frame Data
- **Primary Freeze Frame**: The first freeze frame (associated with first DTC) is erased
- **Secondary Freeze Frame**: Additional freeze frames for subsequent DTCs are cleared
- **Captured Parameters**: All stored sensor values (RPM, speed, coolant temp, O2 voltage, etc.) are deleted
- **Timestamp Data**: Operating condition snapshots when faults occurred are removed

### 4. Emissions Monitor Status
- **Monitor Readiness**: All continuous and non-continuous monitors reset to "not ready" state
- **Monitor Test Results**: Stored test results from Mode 06 are cleared
- **Completion Status**: Monitors must re-run their self-tests during drive cycles
- **Drive Cycle Counters**: Warm-up cycles and similar counters may be reset (implementation-dependent)

### 5. Additional Data Cleared
- **O2 Sensor Test Results**: Mode 05 oxygen sensor monitoring test data (legacy systems)
- **Test Data**: Emissions component test results stored by the ECU
- **Performance Tracking**: Some in-use performance ratios may be affected (Mode 09 PID 08)

## Important Warnings and Considerations

### Does NOT Fix Problems
Mode 04 only clears diagnostic information. It does not:
- Repair faulty sensors or components
- Fix vacuum leaks, misfires, or mechanical issues
- Resolve electrical problems or wiring faults
- Correct fuel system or ignition system problems

If the underlying issue persists, codes will return within 1-3 drive cycles.

### Monitor Readiness Implications
After Mode 04, emissions monitors reset to "not ready":
- **Continuous Monitors**: Misfire, fuel system, and comprehensive component monitors typically become ready within one drive cycle
- **Non-Continuous Monitors**: Catalyst, EVAP, O2 sensor, and secondary air monitors may require specific driving conditions to complete
- **Emissions Testing**: Most jurisdictions require monitors to be "ready" before passing inspection
- **Drive Cycle Requirements**: 50-100 miles of varied driving conditions may be needed for all monitors to complete

### Legal and Compliance Notes
- **Tampering Laws**: Clearing codes to avoid emissions testing may violate Clean Air Act regulations
- **Inspection Failures**: Vehicles with "not ready" monitors may fail emissions inspections
- **Warranty Issues**: Clearing codes before bringing vehicle to dealer may complicate warranty claims
- **Documentation**: Professional technicians should document why Mode 04 was used

### Data Loss
Once Mode 04 is executed, diagnostic data cannot be recovered:
- **Freeze Frames**: Critical diagnostic snapshots are permanently lost
- **Historical DTCs**: Pattern analysis for intermittent faults becomes impossible
- **Time Stamps**: Information about when and how often faults occurred is erased

## When to Use Mode 04

### Appropriate Use Cases

1. **After Successful Repairs**
   - Component replaced or repaired
   - Root cause of DTC has been verified and fixed
   - Ready to verify repair effectiveness through drive cycles

2. **Diagnostic Testing**
   - Clearing intermittent codes to monitor for recurrence
   - Isolating specific fault conditions
   - Verifying repair procedures with fresh diagnostic data

3. **Pre-Sale Inspection Preparation**
   - After emissions-related repairs are completed
   - Preparing vehicle for emissions compliance testing
   - Ensuring all monitors run fresh test cycles

4. **Software Updates**
   - After ECU reflashing or calibration updates
   - Following manufacturer technical service bulletins (TSBs)
   - Resetting monitors after emissions-related software changes

### Inappropriate Use Cases

1. **Avoiding Diagnosis**
   - Clearing codes without identifying root cause
   - Attempting to pass emissions testing without repairs
   - Hiding problems before vehicle sale (unethical/illegal)

2. **Before Data Collection**
   - Clearing codes before retrieving freeze frame data
   - Erasing diagnostic information before proper diagnosis
   - Removing evidence needed for warranty claims

3. **Temporary Fixes**
   - Clearing codes for known persistent issues
   - Using Mode 04 as a "band-aid" for recurring problems
   - Avoiding necessary repairs

## Response Format

### Request Format

Mode 04 uses a simple request format with no additional parameters:

```
CAN ID: 0x7DF (functional/broadcast request)
Data Length: 2 bytes
Byte 0: 0x01 (PCI - length of data)
Byte 1: 0x04 (Mode 04 service ID)
Bytes 2-7: 0x00 (padding)
```

Example request frame:
```
7DF: 01 04 00 00 00 00 00 00
     │  │  └─ Padding bytes
     │  └─ Mode 04
     └─ Length: 1 byte
```

### Response Format

The ECU responds with a positive acknowledgment containing only the mode echo:

```
CAN ID: 0x7E8 (Engine ECU) or 0x7E9-0x7EF (other ECUs)
Data Length: 2 bytes
Byte 0: 0x01 (PCI - length of data)
Byte 1: 0x44 (Mode 04 positive response = 0x04 + 0x40)
Bytes 2-7: 0x00 (padding)
```

Example response frame:
```
7E8: 01 44 00 00 00 00 00 00
     │  │  └─ Padding bytes
     │  └─ Positive response (0x44)
     └─ Length: 1 byte
```

### Multi-ECU Responses

In vehicles with multiple emissions-related ECUs, each ECU that supports Mode 04 will respond:

```
7E8: 01 44 00 00 00 00 00 00  (Engine ECU cleared)
7E9: 01 44 00 00 00 00 00 00  (Transmission ECU cleared)
7EA: 01 44 00 00 00 00 00 00  (Hybrid/Electric ECU cleared)
```

Each ECU independently clears its own diagnostic data.

### Negative Response

If Mode 04 is not supported or fails, the ECU responds with a negative response:

```
7E8: 03 7F 04 XX 00 00 00 00
     │  │  │  │
     │  │  │  └─ Negative Response Code (NRC)
     │  │  └─ Requested Mode (0x04)
     │  └─ Negative Response Service ID (0x7F)
     └─ Length: 3 bytes
```

Common Negative Response Codes (NRC):
- `0x11`: Service Not Supported
- `0x12`: Sub-Function Not Supported
- `0x13`: Incorrect Message Length
- `0x22`: Conditions Not Correct
- `0x31`: Request Out of Range

## Protocol Compliance

### SAE J1979 Requirements

Mode 04 implementation must comply with SAE J1979 (OBD-II Diagnostic Standards):

1. **Mandatory Implementation**: Mode 04 must be supported by all OBD-II compliant vehicles (1996+)
2. **Complete Reset**: All emissions diagnostic data must be cleared, not selectively
3. **Immediate Response**: ECU must respond within 50ms of request reception
4. **MIL Control**: MIL must be turned off immediately upon Mode 04 execution
5. **Monitor Reset**: All readiness monitors must reset to "not ready" state

### ISO 15765-2 (CAN Protocol)

Mode 04 operates over the CAN bus using ISO 15765-2 (ISO-TP) protocol:

1. **Single Frame Transfer**: Mode 04 uses single-frame messaging (no multi-frame needed)
2. **CAN IDs**: Standard OBD-II CAN IDs (0x7DF request, 0x7E8-0x7EF responses)
3. **Baud Rate**: 500 kbps (standard for OBD-II CAN networks)
4. **Frame Format**: 11-bit CAN ID, 8-byte data payload
5. **Priority**: Diagnostic messages use standard CAN priority (not high priority)

### EPA/CARB Compliance

Mode 04 must meet EPA and CARB emissions monitoring requirements:

1. **Monitor Readiness**: Cleared monitors must be detectable via Mode 01 PID 01
2. **Permanent DTCs**: Mode 04 does NOT clear permanent DTCs (Mode 10)
3. **Performance Tracking**: In-use performance ratios may require minimum completion counts
4. **Tampering Prevention**: ECU must log Mode 04 usage (implementation-dependent)

## Implementation in This Simulator

The Teensy 4.0 simulator implements Mode 04 according to OBD-II standards:

### Cleared Data Elements

```cpp
// Clear diagnostic trouble codes
ecu.dtc = false;

// Turn off MIL (Check Engine Light)
digitalWrite(LED_red, LOW);

// Clear both freeze frames
freeze_frame[0].data_stored = false;
freeze_frame[1].data_stored = false;
```

### Response Behavior

- **CAN ID**: Responds on 0x7E8 (Engine ECU)
- **Response Time**: Immediate (< 10ms typical)
- **Format**: Standard single-frame positive response (0x44)
- **Multiple ECUs**: Single ECU simulation (only engine ECU responds)

### Simulator-Specific Notes

1. **Limited Monitor Support**: Basic monitor status implementation (full drive cycle logic not simulated)
2. **Single ECU**: Only engine ECU simulated (no transmission, hybrid, or chassis ECUs)
3. **Simplified DTC Storage**: Binary DTC flag vs. full DTC management system
4. **No Permanent Codes**: Mode 10 (permanent DTCs) not implemented

## Usage Examples

### Example 1: Basic Code Clear with ScanTool

Using a commercial scan tool:

1. Connect scan tool to vehicle OBD-II port
2. Turn ignition to ON position (engine may be running)
3. Select "Clear Codes" or "Erase DTCs" function
4. Scan tool sends: `7DF: 01 04 00 00 00 00 00 00`
5. ECU responds: `7E8: 01 44 00 00 00 00 00 00`
6. MIL turns off immediately
7. Scan tool displays "Codes Cleared Successfully"

### Example 2: Manual CAN Bus Command

Using a CAN bus interface (e.g., CANable, PCAN):

```bash
# Send Mode 04 request
cansend can0 7DF#0104000000000000

# Expected response from Engine ECU
# 7E8: 01 44 00 00 00 00 00 00
```

### Example 3: Python with python-can

```python
import can

# Initialize CAN interface
bus = can.interface.Bus(channel='can0', bustype='socketcan')

# Create Mode 04 request
msg = can.Message(
    arbitration_id=0x7DF,
    data=[0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
    is_extended_id=False
)

# Send request
bus.send(msg)

# Receive response
response = bus.recv(timeout=1.0)
if response and response.arbitration_id == 0x7E8:
    if response.data[1] == 0x44:
        print("Mode 04: DTCs cleared successfully")
    elif response.data[1] == 0x7F:
        print(f"Mode 04: Error - NRC: 0x{response.data[3]:02X}")
```

### Example 4: Arduino/Teensy CAN Library

```cpp
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

void clearDTCs() {
    CAN_message_t msg;

    // Prepare Mode 04 request
    msg.id = 0x7DF;
    msg.len = 8;
    msg.buf[0] = 0x01;  // Length
    msg.buf[1] = 0x04;  // Mode 04
    msg.buf[2] = 0x00;  // Padding
    msg.buf[3] = 0x00;
    msg.buf[4] = 0x00;
    msg.buf[5] = 0x00;
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;

    // Send request
    can1.write(msg);

    // Wait for response
    delay(100);

    if (can1.read(msg)) {
        if (msg.id == 0x7E8 && msg.buf[1] == 0x44) {
            Serial.println("DTCs cleared successfully");
        }
    }
}
```

### Example 5: Post-Repair Workflow

Typical professional technician workflow:

```
1. Diagnose fault using Mode 03 (Read DTCs)
   Request:  7DF: 01 03 00 00 00 00 00 00
   Response: 7E8: 06 43 02 01 18 01 30 00
             (2 DTCs: P0118, P0130)

2. Retrieve freeze frame data (Mode 02)
   Request:  7DF: 02 02 00 00 00 00 00 00
   Response: 7E8: [freeze frame data for DTC]

3. Perform repairs
   - Replace coolant temperature sensor (P0118)
   - Replace O2 sensor (P0130)

4. Clear codes with Mode 04
   Request:  7DF: 01 04 00 00 00 00 00 00
   Response: 7E8: 01 44 00 00 00 00 00 00

5. Verify repair
   - Drive vehicle through complete drive cycle
   - Check Mode 01 PID 01 for monitor readiness
   - Re-scan with Mode 03 to confirm no codes return
```

## Conclusion

Mode 04 is a powerful diagnostic tool that must be used responsibly. It provides a standardized way to reset emissions diagnostic systems, but should only be used after proper diagnosis and repair. Understanding what Mode 04 clears and its implications for monitor readiness is essential for effective vehicle diagnostics and emissions compliance.

## References

- **SAE J1979**: E/E Diagnostic Test Modes (OBD-II Standard)
- **ISO 15765-2**: Road vehicles - Diagnostic communication over Controller Area Network (CAN)
- **SAE J2012**: Diagnostic Trouble Code Definitions
- **EPA 40 CFR Part 86**: Control of Emissions from New and In-Use Highway Vehicles and Engines
- **CARB Title 13, CCR Section 1968.2**: Malfunction and Diagnostic System Requirements

## Related Documentation

- [Mode 01 - Current Powertrain Data](MODE_01.md) - Check monitor readiness after Mode 04
- [Mode 02 - Freeze Frame Data](MODE_02.md) - Data cleared by Mode 04
- [Mode 03 - Read DTCs](MODE_03.md) - DTCs cleared by Mode 04
- [Mode 09 - Vehicle Information](MODE_09.md) - VIN and calibration data (not affected by Mode 04)

---

**Document Version**: 1.0
**Last Updated**: 2025-10-03
**Simulator Version**: Teensy 4.0 OBD-II ECU Simulator
**Implementation File**: `/modes/mode_04.cpp`
