#include "mode_registry.h"

// Define static members
ModeRegistration ModeRegistry::modes[ModeRegistry::MAX_MODES];
uint8_t ModeRegistry::mode_count = 0;
