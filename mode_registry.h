#ifndef MODE_REGISTRY_H
#define MODE_REGISTRY_H

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "ecu_sim.h"

/*
 * OBD-II Mode Handler Registry System
 *
 * This system allows OBD modes to be implemented in separate files
 * and automatically registered at compile time. To add a new mode:
 *
 * 1. Create modes/mode_XX.cpp
 * 2. Implement handle_mode_XX function
 * 3. Add to mode_includes.h
 *
 * The mode will automatically be available in the ECU simulator.
 */

// Forward declarations
extern ecu_t ecu;
extern freeze_frame_t freeze_frame[2];
extern isotp_transfer_t isotp_tx;

/*
 * Mode Handler Function Signature
 *
 * Parameters:
 *   can_MsgRx - Received CAN message with OBD request
 *   can_MsgTx - CAN message to populate with response
 *   ecu_sim   - Reference to ECU simulator for accessing methods
 *
 * Returns:
 *   true  - Mode handled the request
 *   false - Mode did not handle (not this mode's request)
 */
typedef bool (*ModeHandler)(CAN_message_t& can_MsgRx, CAN_message_t& can_MsgTx, ecu_simClass* ecu_sim);

/*
 * Mode Registration Structure
 * Holds information about a registered OBD mode
 */
struct ModeRegistration {
    uint8_t mode_id;           // OBD mode number (e.g., 0x01, 0x09)
    ModeHandler handler;        // Function to call for this mode
    const char* name;          // Human-readable mode name
};

/*
 * Mode Registry Class
 * Manages registration and dispatch of OBD mode handlers
 */
class ModeRegistry {
private:
    static const uint8_t MAX_MODES = 16;  // Support up to 16 modes
    static ModeRegistration modes[MAX_MODES];
    static uint8_t mode_count;

public:
    /*
     * Register a new mode handler
     * Called automatically by mode implementation files
     */
    static bool register_mode(uint8_t mode_id, ModeHandler handler, const char* name) {
        if (mode_count >= MAX_MODES) {
            return false;  // Registry full
        }

        modes[mode_count].mode_id = mode_id;
        modes[mode_count].handler = handler;
        modes[mode_count].name = name;
        mode_count++;

        return true;
    }

    /*
     * Dispatch incoming OBD request to appropriate mode handler
     * Returns true if a mode handled the request
     */
    static bool dispatch(CAN_message_t& can_MsgRx, CAN_message_t& can_MsgTx, ecu_simClass* ecu_sim) {
        uint8_t requested_mode = can_MsgRx.buf[1];

        for (uint8_t i = 0; i < mode_count; i++) {
            if (modes[i].mode_id == requested_mode) {
                return modes[i].handler(can_MsgRx, can_MsgTx, ecu_sim);
            }
        }

        return false;  // No handler found for this mode
    }

    /*
     * Get count of registered modes (for debugging)
     */
    static uint8_t get_mode_count() {
        return mode_count;
    }

    /*
     * Print registered modes (for debugging)
     */
    static void print_registered_modes() {
        Serial.print("Registered OBD Modes: ");
        Serial.println(mode_count);
        for (uint8_t i = 0; i < mode_count; i++) {
            Serial.print("  Mode 0x");
            Serial.print(modes[i].mode_id, HEX);
            Serial.print(": ");
            Serial.println(modes[i].name);
        }
    }
};

/*
 * Helper class for automatic mode registration
 * Usage in mode files:
 *   static ModeRegistrar registrar(0x01, handle_mode_01, "Current Data");
 */
class ModeRegistrar {
public:
    ModeRegistrar(uint8_t mode_id, ModeHandler handler, const char* name) {
        ModeRegistry::register_mode(mode_id, handler, name);
    }
};

#endif // MODE_REGISTRY_H
