#include "command.h"

#include <string.h>

static uint8_t cmd_byte;

void command_performCommand(const uint8_t recv_buf[], const uint32_t recv_len, command_response_t* response)
{
    cmd_byte = recv_buf[0];
    bzero(response->data, RESPONSE_LENGTH);

    switch (cmd_byte)
    {
        case COMMMAND_PING:
            response->length = 2;
            response->data[1] = RESPONSE_OK;
            break;

        case COMMAND_STORE_FLOAT:
            response->length = 3;
            response->data[1] = RESPONSE_FAIL;
            response->data[2] = COMMAND_FAILURE_NOT_IMPLEMENTED;
            break;

        case COMMAND_GET_FLOAT:
            response->length = 3;
            response->data[1] = RESPONSE_FAIL;
            response->data[2] = COMMAND_FAILURE_NOT_IMPLEMENTED;
            break;

        default:
            response->length = 3;
            response->data[1] = RESPONSE_FAIL;
            response->data[2] = COMMAND_FAILURE_NOT_RECOGNIZED;
            break;
    }

    response->data[0] = response->length;
}