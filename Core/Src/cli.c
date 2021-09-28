#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS_CLI.h"
#include <string.h>
#include <stdio.h>

static BaseType_t EchoCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);

static const CLI_Command_Definition_t helpDefinition = 
{
    "echo", // Command string
    "echo: Echos back each parameter on a new line\r\n", // Help string
    EchoCommand, // Command function
    -1 // Number of arg command accepts, -1 indicates inf
};


void RegisterCLICommands(void) 
{
    FreeRTOS_CLIRegisterCommand(&helpDefinition);
}




static BaseType_t EchoCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) 
{
    const char * const firstLine = "Echo:\r\n";
    static size_t currentParameter = 0;

    // If is the first line, print out the command name
    if (currentParameter == 0) {
        strcpy(pcWriteBuffer, firstLine);

        currentParameter = 1;

        return pdTRUE;
    }
    
    BaseType_t parameterLength;
    const char* parameter = 
        FreeRTOS_CLIGetParameter(pcCommandString, currentParameter, &parameterLength);
    
    if (parameter != NULL) {
        memset(pcWriteBuffer, 0, xWriteBufferLen);
        sprintf(pcWriteBuffer, "%d: ", currentParameter);
        strncat(pcWriteBuffer, parameter, (size_t) parameterLength);
        strcat(pcWriteBuffer, "\r\n");

        currentParameter++;
        return pdTRUE;
    }
    else 
    {
        pcWriteBuffer[0] = 0; // Ensure not a valid string
        currentParameter = 0; // Reset for next time command is executed.
        return pdFALSE;
    }
}