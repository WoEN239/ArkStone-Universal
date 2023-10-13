#ifndef ARKSONE_UNIVERSAL_UARTCLIENT_H
#define ARKSONE_UNIVERSAL_UARTCLIENT_H

#include "HardwareSerial.h"
#include "Light.h"
#include "Stopwatch.h"
#include "inttypes.h"

#define UART_MESSAGE_SIZE 2

int8_t uartBuffer[UART_MESSAGE_SIZE];
size_t uartPointer = UART_MESSAGE_SIZE;

bool gamepadActive = false;

StopWatch gamepadTimer = StopWatch();

int8_t gamepadForward = 0;
int8_t gamepadTurn = 0;

void updateUartClient() {
    while (Serial.available()) {
        int8_t rcvByte = Serial.read();
        // Serial.flush();
        if (rcvByte == 127) {  //&& uartPointer == UART_MESSAGE_SIZE)
            uartPointer = 0;
            gamepadActive = true;
            gamepadTimer.reset();
        } /*
     else if(rcvByte == -126 && uartPointer == UART_MESSAGE_SIZE) {
         gamepadForward = map(uartBuffer[0], -125, 125, -100, 100);
         gamepadTurn = map(uartBuffer[1], -125, 125, -100, 100);
         */

        /*
    } else */
        else if (rcvByte == 126 && uartPointer == UART_MESSAGE_SIZE) {
            gamepadForward = uartBuffer[0];
            gamepadTurn = uartBuffer[1];
        } else {
            uartBuffer[uartPointer] = rcvByte;
            if (uartPointer < 2 * UART_MESSAGE_SIZE)
                uartPointer++;
        }
    }
    if (gamepadTimer.milliseconds() > 500.0)
        gamepadActive = false;
}

#endif  // ARKSONE_UNIVERSAL_UARTCLIENT_H