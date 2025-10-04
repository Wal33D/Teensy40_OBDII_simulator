# OBD-II Mode 03 - Request Emissions-Related Diagnostic Trouble Codes

## Overview

Mode 03 is one of the core OBD-II diagnostic modes that provides access to stored emissions-related Diagnostic Trouble Codes (DTCs). These are confirmed, "matured" fault codes that have triggered the Malfunction Indicator Lamp (MIL), commonly known as the "Check Engine Light."

**Mode Code**: `0x03`
**Response Code**: `0x43`
**Request Format**: `[Length] [0x03]`
**Response Format**: `[Length] [0x43] [DTC Count] [DTC1 High] [DTC1 Low] [DTC2 High] [DTC2 Low] ...`

## Purpose in OBD-II

Mode 03 is a critical emissions monitoring function mandated by the EPA and CARB for OBD-II compliance. Its primary purposes are:

1. **Emissions Compliance Verification**: Retrieve codes related to emissions system failures
2. **MIL Status Reporting**: Return only DTCs that have triggered the Check Engine Light
3. **Fault Diagnosis**: Provide technicians with confirmed fault codes for emissions-related issues
4. **Regulatory Requirements**: Enable emissions testing stations to verify vehicle compliance

### What Mode 03 Returns

- **Confirmed DTCs Only**: Codes that have "matured" through multiple drive cycles
- **Emissions-Related Only**: Powertrain codes that affect emissions (primarily "P0" and "P2" codes)
- **MIL-Associated Only**: DTCs that have illuminated the Check Engine Light
- **Current State**: Active faults stored in ECU memory

### What Mode 03 Does NOT Return

- **Pending DTCs**: Use Mode 07 for codes detected but not yet confirmed
- **Permanent DTCs**: Use Mode 10 for codes that persist after clearing
- **Non-Emissions Codes**: Body (B), Chassis (C), or Network (U) codes are manufacturer-specific
- **History Codes**: Previously cleared or resolved DTCs

## DTC Format Explanation

OBD-II uses a standardized 5-character alphanumeric format for Diagnostic Trouble Codes. Understanding this format is essential for proper diagnosis.

### DTC Code Categories

The first character indicates the system affected:

- **P**: Powertrain (Engine, Transmission, Emissions) - OBD-II standardized
- **C**: Chassis (ABS, Suspension, Steering) - Manufacturer-specific
- **B**: Body (Airbags, Climate, Lighting) - Manufacturer-specific
- **U**: Network (CAN Bus, Communication) - Manufacturer-specific

**Note**: Mode 03 returns only **P-codes** related to emissions monitoring.

### P-Code Subcategories

The second digit further categorizes powertrain codes:

- **P0xxx**: Generic OBD-II codes (standardized across all manufacturers)
- **P1xxx**: Manufacturer-specific codes
- **P2xxx**: Generic OBD-II codes (standardized)
- **P3xxx**: Manufacturer-specific codes

### Complete DTC Format

```
P 0 1 0 0
│ │ │ └─┴─ Specific fault identifier (00-99)
│ │ └───── System affected (1 = Fuel/Air metering)
│ └─────── Code type (0 = Generic, 1 = Manufacturer)
└───────── Powertrain system
```

### Binary Encoding in CAN Messages

Each DTC is transmitted as 2 bytes in the CAN response:

**Byte 1 (High Byte)**:
- Bits 7-6: DTC type (00=P0xxx, 01=P1xxx, 10=P2xxx, 11=P3xxx/U)
- Bits 5-4: First digit of code (0-3)
- Bits 3-0: Second digit of code (0-9)

**Byte 2 (Low Byte)**:
- Bits 7-4: Third digit of code (0-9)
- Bits 3-0: Fourth digit of code (0-9)

#### Encoding Examples

**P0100** (MAF Circuit Malfunction):
```
High Byte: 0x01 = 0000 0001
           ││││ ││││
           ││└┴─┴┴─┴─ 0001 = First digit "1"
           └┴──────── 00 = P0xxx type

Low Byte:  0x00 = 0000 0000
           ││││ ││││
           │└─┴─┴┴─┴─ 0000 = Fourth digit "0"
           └────────── 0000 = Third digit "0"

Result: P0100
```

**P0200** (Injector Circuit Malfunction):
```
High Byte: 0x02 = 0000 0010
Low Byte:  0x00 = 0000 0000
Result: P0200
```

## Simulated DTCs

This simulator implements two common emissions-related diagnostic trouble codes that represent critical sensor failures affecting emissions compliance.

### P0100 - Mass Air Flow (MAF) Circuit Malfunction

**System**: Fuel and Air Metering
**Component**: Mass Air Flow Sensor
**Severity**: High - Direct impact on fuel delivery and emissions

**Description**:
The MAF sensor measures the volume and density of air entering the engine. This measurement is critical for calculating the correct fuel injection quantity to maintain the stoichiometric air/fuel ratio (14.7:1) required for proper catalytic converter operation and minimal emissions.

**Emissions Impact**:
- **Fuel Mixture**: ECU cannot accurately calculate required fuel, leading to rich or lean conditions
- **Catalyst Damage**: Incorrect air/fuel ratios can overheat and damage the catalytic converter
- **Increased Emissions**: Poor combustion increases HC, CO, and NOx emissions
- **Fuel Economy**: Engine may run excessively rich, wasting fuel and increasing CO2

**Common Causes**:
- Contaminated or failed MAF sensor element
- Air leaks between MAF sensor and throttle body
- Damaged wiring or connector to MAF sensor
- Clogged or dirty air filter restricting airflow

**Freeze Frame Data**:
When P0100 is set, the ECU captures operating conditions including engine RPM, vehicle speed, coolant temperature, throttle position, and O2 sensor readings for diagnostic purposes.

### P0200 - Injector Circuit Malfunction

**System**: Fuel and Air Metering
**Component**: Fuel Injector Circuit (Bank-wide)
**Severity**: High - Complete fuel system failure possible

**Description**:
The fuel injector circuit DTC indicates an electrical problem with one or more fuel injectors or their control circuits. Fuel injectors are precision solenoids that meter fuel into the engine cylinders. Proper injector operation is essential for emissions compliance.

**Emissions Impact**:
- **Incomplete Combustion**: Failed injectors cause misfires, dramatically increasing HC emissions
- **Catalyst Damage**: Unburned fuel entering exhaust can overheat catalytic converter
- **O2 Sensor Contamination**: Raw fuel can damage oxygen sensors
- **Excessive Emissions**: Misfires produce high levels of hydrocarbons and carbon monoxide

**Common Causes**:
- Failed fuel injector solenoid
- Shorted or open injector wiring
- Damaged ECU injector driver circuit
- Low fuel pressure affecting all injectors
- Contaminated fuel clogging injector nozzles

**Freeze Frame Data**:
Operating conditions at the moment of detection help identify whether the fault occurred during idle, acceleration, or cruise conditions.

## MIL (Malfunction Indicator Lamp) Relationship

The Malfunction Indicator Lamp (MIL), commonly called the "Check Engine Light," is the primary emissions warning to the driver.

### MIL Illumination Logic

Mode 03 DTCs have a direct relationship with the MIL:

1. **Detection**: ECU detects an emissions-related fault (e.g., MAF sensor failure)
2. **Confirmation**: Fault persists for specified conditions (varies by code)
3. **DTC Storage**: Code is stored in non-volatile memory
4. **MIL Activation**: Check Engine Light illuminates on dashboard
5. **Mode 03 Availability**: DTC becomes available via Mode 03 requests

### Two-Trip DTC Logic

Most emissions DTCs follow EPA-mandated "two-trip" logic:

**Trip 1**: Fault detected and pending DTC set (Mode 07)
**Trip 2**: Fault confirmed, DTC matures to Mode 03, MIL illuminates

**Drive Cycle Definition**: Engine started cold, driven until warm, then shut off

### MIL States in This Simulator

The simulator models MIL behavior through a physical LED and software flag:

**No DTCs (ecu.dtc == 0)**:
- Red LED: OFF
- MIL Status: Not illuminated
- Mode 03 Response: 0 DTCs stored
- System Status: All emissions systems operating normally

**DTCs Present (ecu.dtc == 1)**:
- Red LED: ON (represents illuminated Check Engine Light)
- MIL Status: Illuminated
- Mode 03 Response: 2 DTCs stored (P0100, P0200)
- System Status: Emissions system malfunction detected

**Triggering DTCs**: Press SW1 button to toggle between states

### MIL Extinguishing

The MIL will turn off after:
1. **Repair Complete**: Fault condition is fixed
2. **Confirmation**: ECU verifies repair through multiple drive cycles (typically 3)
3. **Self-Clear**: Some DTCs self-clear after 40 warm-up cycles without recurrence

**Manual Clear**: Mode 04 (Clear DTCs) immediately turns off MIL and erases codes

## Response Format

Mode 03 uses a simple, structured response format that scales with the number of stored DTCs.

### No DTCs Stored Response

When no emissions faults are present:

```
CAN ID: 0x7E8 (Engine ECU Response)
Length: 8 bytes
Data:   [02] [43] [00] [00] [00] [00] [00] [00]
         │    │    │    └─────────────────── Padding
         │    │    └────────────────────────── 0 DTCs stored
         │    └─────────────────────────────── Mode 03 response
         └──────────────────────────────────── Length: 2 data bytes
```

**Interpretation**:
- No emissions faults detected
- MIL should be OFF
- Vehicle is in emissions compliance
- No diagnostic action required

### DTCs Present Response

When emissions faults are stored:

```
CAN ID: 0x7E8 (Engine ECU Response)
Length: 8 bytes
Data:   [06] [43] [02] [01] [00] [02] [00] [00]
         │    │    │    │    │    │    │    │
         │    │    │    │    │    │    │    └─── Padding
         │    │    │    │    │    └────┴────── DTC2: P0200
         │    │    │    └────┴────────────────── DTC1: P0100
         │    │    └─────────────────────────── 2 DTCs stored
         │    └──────────────────────────────── Mode 03 response
         └───────────────────────────────────── Length: 6 data bytes
```

**Interpretation**:
- 2 emissions faults confirmed
- MIL should be illuminated (Check Engine Light ON)
- P0100: MAF Circuit Malfunction
- P0200: Injector Circuit Malfunction
- Immediate diagnostic attention required

### Multi-DTC Handling

For more than 2 DTCs, the response extends using ISO-TP multi-frame protocol:

**First Frame** (up to 3 DTCs):
```
[10] [0A] [43] [03] [01] [00] [02] [00]
 │    │    │    │    └──────────────── DTC1
 │    │    │    └───────────────────── 3 DTCs
 │    │    └────────────────────────── Mode response
 │    └─────────────────────────────── Total length: 10 bytes
 └──────────────────────────────────── First frame indicator
```

**Flow Control** (from scanner):
```
[30] [00] [00] ...
 │    │    └────── Separation time: 0ms
 │    └─────────── Block size: unlimited
 └──────────────── Continue sending
```

**Consecutive Frame**:
```
[21] [03] [00] [00] [00] [00] [00] [00]
 │    └──────────────────────────────── DTC2, DTC3, padding
 └─────────────────────────────────────── Sequence #1
```

## Protocol Compliance

This Mode 03 implementation adheres to industry standards for emissions diagnostics.

### Standards Conformance

**ISO 15765-4 (ISO-TP)**:
- CAN bus physical layer (500 kbps, 11-bit identifiers)
- Standard diagnostic addressing (0x7DF request, 0x7E8 response)
- Single-frame response support for 0-2 DTCs
- Multi-frame capability for >2 DTCs (extensible)

**SAE J1979**:
- Mode 03 service definition
- DTC format encoding (2 bytes per code)
- Response format structure
- Positive response handling

**EPA/CARB Requirements**:
- Returns only confirmed, emissions-related DTCs
- DTCs correspond to MIL illumination
- Supports emissions testing station diagnostics
- Maintains DTC data persistence

### CAN Bus Configuration

**Request Message**:
- CAN ID: `0x7DF` (functional broadcast) or `0x7E0` (physical to engine)
- DLC: 8 bytes
- Data: `[02] [03] [00] [00] [00] [00] [00] [00]`

**Response Message**:
- CAN ID: `0x7E8` (engine ECU response)
- DLC: 8 bytes
- Data: Variable based on DTC count
- Timing: <50ms from request (typical ECU response time)

### Error Handling

**Invalid Request**:
- Malformed Mode 03 requests are ignored
- No negative response required by standard
- Other mode handlers attempt processing

**No Response Conditions**:
- ECU not powered or in sleep mode
- CAN bus error (no acknowledgment)
- Request addressed to different ECU (0x7E1-0x7E7)

## Usage Examples

### Example 1: Checking for DTCs (No Faults)

**Scenario**: Technician checks for emissions codes on a healthy vehicle

**Request** (from scan tool):
```
CAN ID: 0x7DF
Data:   [02] [03] [00] [00] [00] [00] [00] [00]
```

**Response** (from ECU):
```
CAN ID: 0x7E8
Data:   [02] [43] [00] [00] [00] [00] [00] [00]
```

**Scan Tool Display**:
```
Mode 03 - Request DTCs
Status: No DTCs stored
MIL Status: OFF
Result: PASS - Vehicle in emissions compliance
```

### Example 2: Retrieving Active DTCs

**Scenario**: Check Engine Light is ON, driver brings vehicle for diagnosis

**Request** (from scan tool):
```
CAN ID: 0x7DF
Data:   [02] [03] [00] [00] [00] [00] [00] [00]
```

**Response** (from ECU):
```
CAN ID: 0x7E8
Data:   [06] [43] [02] [01] [00] [02] [00] [00]
```

**Scan Tool Display**:
```
Mode 03 - Request DTCs
DTC Count: 2
DTCs Found:
  P0100 - Mass Air Flow Circuit Malfunction
  P0200 - Injector Circuit Malfunction
MIL Status: ON
Result: FAIL - Emissions system malfunction
```

**Technician Action**:
1. Request freeze frame data (Mode 02) for operating conditions at fault
2. Inspect MAF sensor for contamination or damage
3. Test injector circuits for continuity and resistance
4. Repair faults and verify with test drive
5. Clear codes (Mode 04) after confirming repair

### Example 3: Simulator Physical Interaction

**Scenario**: Developer testing OBD-II scanner software

**Initial State**:
```
SW1 Button: Not pressed
Red LED: OFF
ecu.dtc: 0
Mode 03 Response: No DTCs
```

**Action**: Press SW1 button on Teensy shield

**New State**:
```
SW1 Button: Pressed (falling edge detected)
Red LED: ON (simulates MIL illumination)
ecu.dtc: 1
Freeze Frame: Captured (current engine conditions)
Mode 03 Response: 2 DTCs (P0100, P0200)
```

**Subsequent Mode 03 Request**:
```
Request:  [02] [03] [00] [00] [00] [00] [00] [00]
Response: [06] [43] [02] [01] [00] [02] [00] [00]
```

**Action**: Press SW1 button again

**Final State**:
```
SW1 Button: Pressed (falling edge detected)
Red LED: OFF (MIL extinguished)
ecu.dtc: 0
Mode 03 Response: No DTCs
```

This toggle behavior allows developers to test both fault and no-fault scenarios without requiring actual sensor failures.

### Example 4: Emissions Testing Station Workflow

**Scenario**: Annual emissions inspection using OBD-II readiness check

**Step 1 - Check for Active DTCs**:
```
Request Mode 03
Response: [02] [43] [00] ...
Result: No DTCs present
```

**Step 2 - Check Monitor Readiness** (Mode 01 PID 01):
```
Request: [02] [01] [01] ...
Response: Shows monitor completion status
Result: All monitors ready
```

**Step 3 - Verify MIL Status**:
```
Physical: Check dashboard for MIL
Simulator: Red LED OFF
Result: MIL not commanded ON
```

**Inspection Outcome**: PASS - Vehicle meets emissions requirements

**If DTCs Were Present**:
```
Response: [06] [43] [02] [01] [00] [02] [00] [00]
Result: FAIL - Emissions malfunction detected
Action: Repairs required, retest after Mode 04 clear and drive cycle
```

## Integration with Other Modes

Mode 03 works in conjunction with other OBD-II modes for comprehensive emissions diagnostics:

**Mode 01 PID 01** - Monitor Status:
- Shows MIL state (ON/OFF)
- Indicates number of DTCs stored
- Cross-reference with Mode 03 DTC count

**Mode 02** - Freeze Frame Data:
- Retrieves operating conditions when DTC was set
- Uses same PID structure as Mode 01
- Essential for diagnosing intermittent faults

**Mode 04** - Clear DTCs:
- Erases all Mode 03 stored codes
- Turns off MIL
- Resets freeze frame data
- Resets emissions monitor readiness

**Mode 07** - Pending DTCs:
- Shows codes detected but not yet confirmed
- Helps identify intermittent issues before MIL illuminates
- One-trip codes awaiting second trip confirmation

**Mode 10** - Permanent DTCs:
- Codes that cannot be cleared with Mode 04
- Require actual repair and drive cycle self-clearing
- Not implemented in this simulator

## Implementation Notes

This simulator's Mode 03 implementation is located in `/modes/mode_03.cpp` and uses the modular mode registration architecture.

**File**: `/modes/mode_03.cpp` (89 lines)

**Key Features**:
- Self-contained mode handler with automatic registration
- Uses global `ecu.dtc` flag (0=no faults, 1=faults present)
- Returns 2 hardcoded DTCs (P0100, P0200) when `ecu.dtc == 1`
- Single-frame response (fits in one CAN message)
- Immediate response without ISO-TP multi-frame overhead

**Extensibility**:
To add more DTCs or dynamic fault detection:
1. Expand `ecu.dtc` to a bitmask or array
2. Implement per-DTC enable/disable logic
3. Add ISO-TP multi-frame support for >2 DTCs
4. Integrate with Mode 01 monitor status for automatic DTC maturation

**Hardware Interface**:
- SW1 button toggles `ecu.dtc` flag
- Red LED reflects MIL state
- Freeze frame data captured on DTC set (for Mode 02)

---

## Further Reading

- **SAE J1979**: OBD-II Scan Tool Standard
- **ISO 15765-4**: CAN Bus Diagnostic Communication
- **EPA CFR Title 40 Part 86**: Emissions Testing Procedures
- **ISO 14229**: Unified Diagnostic Services (UDS)

---

**Document Version**: 1.0
**Last Updated**: 2024
**Simulator Version**: v3.0 (Modular Architecture)
**Author**: Waleed Judah (Wal33D)
