#ifndef DEFINITIONS_RELAYS_H
#define DEFINITIONS_RELAYS_H

// Relays number

#define RELAY_K1                0x40 // Designator for relay K1
#define RELAY_K2                0x41 // Designator for relay K2
#define RELAY_K3                0x42 // Designator for relay K3
#define RELAY_K4                0x43 // Designator for relay K4 

// Main control commands

#define CMD_OPEN                1    // Open a relay. pass relay number in param. No option.
#define CMD_CLOSE               2    // Close a relay. pass relay number in param. No option.
#define CMD_TOGGLE              3    // Toggle a relay. pass relay number in param. No option.
#define CMD_CLOSE_PULSE         4    // Close a relay. pass relay number in param. Pulse duration in opt or leave 0 for default.
#define CMD_OPEN_PULSE          5    // Open a relay. pass relay number in param. Pulse duration in opt or leave 0 for default.
#define CMD_CLOSE_ALL_RELAYS    6    // Close all relays. No option.
#define CMD_OPEN_ALL_RELAYS     7    // Open all relays. No option.

// Configuration and emergency off

#define SET_PULSE_DURATION      8    // Set pulse duration in seconds (1 ... 4294967296)
#define SET_ENABLE_LOCAL_CTRL   9    // Enable coil open/close from board. No option.
#define SET_DISABLE_LOCAL_CTRL  10   // Disable coil open/close from board. No option.
#define SET_ENABLE_REMOTE_CTRL  11   // Enable coil open/close from remote. No option.
#define SET_DISABLE_REMOTE_CTRL 12   // Disable coil open/close from remote. No option.
#define SET_EMERGENCY_OFF       13   // Open all coils and locks board. Requires local reset to restart operation.

// Read-back

#define READ_RELAY_POSITION     23   // Reads relay position (1 for open, 2 for close)
#define READ_STATUS             24   // Reads board status
#define READ_PORT               25   // Reads board port (PA7-0 k1 k2 k3 k4 s1 s2 s3 s4)

#define RELAY_IS_OPEN           1    // Returned value if coil is open.
#define RELAY_IS_CLOSED         2    // Returned value if coil is closed.

// Status

#define STATUS_BITS_AVAILABLE   0    // Board in available
#define STATUS_BITS_COM_ERROR   1    // A serial communication error occured
#define STATUS_BITS_LOC_CTRL_EN 2    // Local control enabled
#define STATUS_BITS_REM_CTRL_EN 3    // Remote control enabled
#define STATUS_BITS_IS_EOFF     4    // Is in Emergency Off state (local reset required)

// Common definitions

#define COMMON_READ_TYPE        0x80 // Return device tpye
#define COMMON_READ_STATUS      0x81 // Return device status
#define COMMON_READ_WARNINGS    0x82 // Return device warnings
#define COMMON_READ_ERRORS      0x83 // Return device errors

#endif