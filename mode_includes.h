#ifndef MODE_INCLUDES_H
#define MODE_INCLUDES_H

/*
 * OBD-II Mode Includes
 *
 * This file includes all mode implementation files.
 * To add a new mode:
 * 1. Create modes/mode_XX.cpp
 * 2. Add #include for the .cpp file below
 * 3. The mode will automatically register itself
 *
 * Note: We include .cpp files directly because Arduino IDE
 * requires explicit includes for compilation.
 */

#include "modes/mode_01.cpp"  // Current Powertrain Data
#include "modes/mode_02.cpp"  // Freeze Frame Data
#include "modes/mode_03.cpp"  // Request Emissions DTCs
#include "modes/mode_04.cpp"  // Clear Emissions Diagnostic Info
#include "modes/mode_09.cpp"  // Vehicle Information

#endif // MODE_INCLUDES_H
