#ifndef COMMAND_H
#define COMMAND_H

#include <stdint.h>

#define RESPONSE_LENGTH 6
#define RESPONSE_OK 0x42
#define RESPONSE_FAIL 0x12

// Ping ESP32
// Response: 0x42
#define COMMMAND_PING 0x00

// Send a float to be stored on ESP32
// Response: 0x42 ok followed by value, 0x12 fail followed by error code
#define COMMAND_STORE_FLOAT 0x01

// Get a float stored on ESP32 flash
// Response: 0x42 ok followed by value, 0x12 fail followed by error code
#define COMMAND_GET_FLOAT 0x02

#define COMMAND_FAILURE_NOT_IMPLEMENTED 0x00
#define COMMAND_FAILURE_NOT_RECOGNIZED 0x01

typedef struct {
    uint8_t length;
    uint8_t data[RESPONSE_LENGTH];
} command_response_t;

void command_performCommand(const uint8_t recv_buf[], const uint32_t recv_len, command_response_t* response);

#endif