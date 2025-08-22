#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "debug.h"
#include "deck.h"
#include "uart2.h"
#include "log.h"
#include "param.h"
#include "FreeRTOS.h"
#include "task.h"

#define BAUDRATE 115200
#define MAX_STR_LEN 127

// Logging buffer and parsed variables
static char uartString[MAX_STR_LEN] = {0};
static int16_t flowX = 0;
static int16_t flowY = 0;

static void windSensorTask(void *param) {
    char rxBuffer[MAX_STR_LEN] = {0};
    int idx = 0;

    while (1) {
        uint8_t c;
        //DEBUG_PRINT("UART Active!\n");
        if (uart2GetDataWithTimeout(1, &c, 10)) {

            //DEBUG_PRINT("Received char: %c\n", c);

            if ((c == '\n' || c == '\r') && idx > 0) {
                rxBuffer[idx < MAX_STR_LEN ? idx : MAX_STR_LEN - 1] = '\0';
                strncpy(uartString, rxBuffer, MAX_STR_LEN);

                // DEBUG_PRINT("Parsed line: %s\n", uartString);

                char *token = strtok(uartString, " ");
                int tokenIdx = 0;

                while (token != NULL) {
                    switch (tokenIdx) {
                        case 1:
                            flowX = atoi(token);
                            break;
                        case 0:
                            flowY = -atoi(token);
                            break;

                    }
                    token = strtok(NULL, " ");
                    tokenIdx++;
                }

                idx = 0;
                memset(rxBuffer, 0, MAX_STR_LEN);
            }
            else {
                if (idx < MAX_STR_LEN - 1) {
                    rxBuffer[idx++] = (char)c;
                }

                    //  Fix 3: Buffer overflow guard (in case line is too long or never ends)
                if (idx >= MAX_STR_LEN - 1) {
                    // DEBUG_PRINT("Buffer overflow or no newline — clearing buffer\n");
                    idx = 0;
                    memset(rxBuffer, 0, MAX_STR_LEN);
                }
            }
        }
        //vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static void deck_windSensorInit(struct deckInfo_s *info) {
    uart2Init(BAUDRATE);
    xTaskCreate(windSensorTask, "windSensor", 256, NULL, 3, NULL);
}

static bool helloUart(void) {
    // DEBUG_PRINT("UART Active!\n");
    return true;
}

const DeckDriver windSensor = {
    .name = "windSensor",
    .init = deck_windSensorInit,
    .test = helloUart,
};

DECK_DRIVER(windSensor);

// Logging group for x, y, and gas values
LOG_GROUP_START(windSensor)
LOG_ADD(LOG_INT16, flowX, &flowX)
LOG_ADD(LOG_INT16, flowY, &flowY)
LOG_GROUP_STOP(windSensor)
