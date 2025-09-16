/*
 * cli.h
 *
 *  Created on: Aug 11, 2025
 *      Author: tilen
 */

#ifndef INC_CLI_H_
#define INC_CLI_H_

#include "main.h"

#include "usb_device.h"      // Include the USB Device header
#include "usbd_cdc_if.h"     // Include the USB CDC interface header
#include <string.h>  // For string functions like strcmp
#include <math.h>



// Define command structure
typedef struct {
    char *command;                      // Command string
    int32_t (*handler)(void *args);         // Handler function for the command
} CLI_Command_t;

// Define command maximum length
#define MAX_CMD_LENGTH 50


// Function prototypes
int32_t HandlePing(void *args);
int32_t HandleChangeFreq(void * freq);
int32_t HandleChangeAmp(void* args);
HAL_StatusTypeDef cli_processCommand(uint8_t *command);
HAL_StatusTypeDef cli_sendData(uint8_t *data, uint16_t length);








#endif /* INC_CLI_H_ */
