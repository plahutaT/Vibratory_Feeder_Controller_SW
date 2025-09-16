/*
 * cli.c
 *
 *  Created on: Aug 11, 2025
 *      Author: tilen
 *
 *  Description: This file implements a simple Command-Line Interface (CLI) for the STM32H7 microcontroller project.
 *  The CLI processes commands received over USB Virtual COM Port (VCP) and executes corresponding handlers.
 *  Supported commands:
 *  - "PING": Sends a response to verify communication.
 *  - "CHANGE_FREQ": Changes the output sine wave frequency by updating TIM3's auto-reload register (ARR).
 *  - "CHANGE_AMP": Changes the amplitude of the sine wave by scaling the inactive DMA buffer and requesting a buffer swap.
 *  Commands are parsed from a buffer, and arguments are extracted after a space delimiter.
 *  Responses are sent back via USB VCP using CDC_Transmit_FS.
 *  This CLI is used to dynamically adjust SPWM parameters without halting the system.
 *  Dependencies: usbd_cdc_if.h for USB VCP transmission, string.h for string operations.
 *  Global variables: Extern declarations for TIM3_Ticks, htim3, LUT buffers, and DMA flags used in handlers.
 *  Limitations: Command names are case-sensitive, arguments must be integers, no error handling for non-numeric args.
 *  Usage: Call cli_processCommand with the received USB buffer in the main loop.
 */


#include "cli.h"
#include "spwm.h"  // Central SPWM LUT generation

/* Ensure standard integer types and math constants available */
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

extern uint32_t TIM3_Ticks;          // Global variable for TIM3 period ticks, updated for frequency changes.
extern TIM_HandleTypeDef htim3;      // Handle for TIM3, used to set ARR for frequency adjustment.

extern volatile uint32_t g_ns_active;
extern volatile uint32_t g_ns_inactive;
extern uint16_t *pspwm_active;                 // Active sine buffer (leg A)
extern uint16_t *pspwm_active_inv;             // Active inverse buffer (leg B)
extern uint16_t *pspwm_inactive;               // Inactive sine buffer
extern uint16_t *pspwm_inactive_inv;           // Inactive inverse buffer
extern volatile uint8_t spwm_swap_request_f;   // Swap request flag (handled in DMA Cplt)
extern volatile uint8_t spwm_start_request_f;   // Swap request flag (handled in DMA Cplt)
extern TIM_HandleTypeDef htim1;                // For ARR access

/* Carrier frequency constant (matches F_CARRIER in spwm.h) */
#define SPWM_F_CARRIER_HZ ((float)F_CARRIER)

/* Track current runtime settings */
static uint32_t g_amp_percent = 100U;  /* 0..100 */
static uint32_t g_freq_hz = 50U;       /* Default fundamental (adjust to your initial config) */

// Define supported commands and their handlers
CLI_Command_t cli_commands[] = {
    { "PING", HandlePing },
    { "CHANGE_FREQ", HandleChangeFreq },
    { "CHANGE_AMP", HandleChangeAmp },
};

// Handler for "PING" command
// Args: Not used.
// Sends a fixed response string over USB VCP to confirm communication.
// Returns HAL_OK on success, HAL_ERROR on transmission failure.


int32_t HandlePing(void *args)
{
	char response[30];

    sprintf(response, "HELLO_FROM_STM\n");
    if (CDC_Transmit_FS((uint8_t*)response, (uint16_t)strlen(response)) != HAL_OK)
    {
    	return HAL_ERROR;
    }

    return HAL_OK;
}



/* Frequency change: rebuild BOTH active and inactive buffers (size may change) */
int32_t HandleChangeFreq(void* args)
{
    char response[96];
    if (args == NULL) {
        sprintf(response, "ERR: No frequency\n");
        CDC_Transmit_FS((uint8_t*)response, strlen(response));
        return HAL_ERROR;
    }

    uint32_t newFreq = (uint32_t)atoi((char*)args);
    if (newFreq < 20 || newFreq > 200) {
        sprintf(response, "ERR: Range 20-200\n");
        CDC_Transmit_FS((uint8_t*)response, strlen(response));
        return HAL_ERROR;
    }

    uint16_t arr = (uint16_t)htim1.Init.Period;

    /* Mirror into inactive (so next amp change has a base to scale) */
    uint32_t new_ns_active = 0;
    if (SPWM_GenerateLUTs_Inactive(SPWM_F_CARRIER_HZ, (float)newFreq, arr,
                                   (float)g_amp_percent / 100.0f, &new_ns_active) != 0) {
        sprintf(response, "ERR: Inactive gen\n");
        CDC_Transmit_FS((uint8_t*)response, strlen(response));
        return HAL_ERROR;
    }

    g_ns_inactive = new_ns_active;
    g_freq_hz = newFreq;

    sprintf(response, "OK: F=%luHz ns=%lu (restart)\n",
            (unsigned long)g_freq_hz, (unsigned long)g_ns_inactive);
    CDC_Transmit_FS((uint8_t*)response, strlen(response));

    spwm_swap_request_f = 1;
    return HAL_OK;
}

/* Amplitude change: only regenerate INACTIVE buffers; request swap on next DMA completion */
int32_t HandleChangeAmp(void* args)
{
    char response[80];
    if (args == NULL) {
        sprintf(response, "ERR: No amp\n");
        CDC_Transmit_FS((uint8_t*)response, strlen(response));
        return HAL_ERROR;
    }
    int ampPercent = atoi((char*)args);
    if (ampPercent < 0 || ampPercent > 100) {
        sprintf(response, "ERR: Amp 0-100\n");
        CDC_Transmit_FS((uint8_t*)response, strlen(response));
        return HAL_ERROR;
    }

    uint16_t arr = (uint16_t)htim1.Init.Period;
    uint32_t tmp_ns = 0;
    if (SPWM_GenerateLUTs_Inactive(SPWM_F_CARRIER_HZ, (float)g_freq_hz, arr,
                                   (float)ampPercent / 100.0f, &tmp_ns) != 0) {
        sprintf(response, "ERR: Gen inactive\n");
        CDC_Transmit_FS((uint8_t*)response, strlen(response));
        return HAL_ERROR;
    }

    if (tmp_ns != g_ns_active) {
        /* Should never differ; protect against accidental freq-based NS change here */
        sprintf(response, "ERR: ns diff (abort)\n");
        CDC_Transmit_FS((uint8_t*)response, strlen(response));
        return HAL_ERROR;
    }
    g_ns_inactive = tmp_ns;
    g_amp_percent = (uint32_t)ampPercent;


    sprintf(response, "OK: Amp=%d%% swap\n", ampPercent);
    CDC_Transmit_FS((uint8_t*)response, strlen(response));

    spwm_swap_request_f = 1;
    return HAL_OK;
}



// Function to process received command
// Command: Pointer to received command buffer (null-terminated string).
// Parses the command name and arguments (split by space).
// Removes \r\n from command.
// Looks up command in cli_commands array and calls the handler with args.
// Sends "Invalid command" if no match.
// Returns HAL_OK on success, HAL_ERROR on transmission failure.

HAL_StatusTypeDef cli_processCommand(uint8_t *command)
{
    char response[50];
    char command_name[20];
    uint8_t *args = NULL;

    // Remove any newline or carriage return characters from the received command
    command[strcspn((char*)command, "\r\n")] = 0;

    // Split command and arguments
    args = (uint8_t*)strchr((char*)command, ' ');
    if (args != NULL) {
        // Terminate command_name at the space, and advance args pointer
        strncpy(command_name, (char*)command, args - command);
        command_name[args - command] = '\0';
        args++;  // Point to the first character of the argument part
    } else {
        // No arguments, command_name is the entire command
        strcpy(command_name, (char*)command);
    }

    // Look for a matching command
    for (int i = 0; i < sizeof(cli_commands) / sizeof(CLI_Command_t); i++) {
        if (strcmp(command_name, cli_commands[i].command) == 0) {
            // Call the handler for the matched command with arguments
            cli_commands[i].handler(args);
            return HAL_OK;
        }
    }

    // If no match is found
    sprintf(response, "Invalid command\n");
    if (CDC_Transmit_FS((uint8_t*)response, (uint16_t)strlen(response)) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}





/*********************************** Helper Functions **************************************/



