//
// Created by oaleksander on 07.04.2023.
//

#ifndef ARKSTONE_UNIVERSAL_UARTCOLORSENSOR_H
#define ARKSTONE_UNIVERSAL_UARTCOLORSENSOR_H

#include "SoftwareSerial.h"
#include "Constants.h"

#define UARTCOLORSENSOR_PACKET_SIZE 3
#define UARTCOLORSENSOR_TERMINATOR '\n'

typedef struct __attribute__ ((packed)) UartColorData {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} UartColorData;

uint8_t nSoftwareSerialInstances = 0;
uint8_t softwareSerialInstanceToListen = 0;
#define SOFTWARE_SERIAL_INSTANCE_SWITCH_TIMEOUT 300

class UartColorSensor {
private:
    SoftwareSerial *uartHandle;
    size_t uartColorSensorBufferPointer = 0;
    uint8_t nThisInstance;
    uint32_t lastTimeThisInstanceChosenMs = 0;

    void incrementSoftwareSerialInstanceChoice() {
        softwareSerialInstanceToListen++;
        if (softwareSerialInstanceToListen > nSoftwareSerialInstances)
            softwareSerialInstanceToListen = 1;
        this->uartHandle->flush();
#ifdef SERIAL_DEBUG
        Serial.println("SS sw");
#endif
    }

public:
    UartColorData dataOutput;
    UartColorData dataBuffer;

    UartColorSensor(uint8_t portNumber) {
        this->uartHandle = new SoftwareSerial(portNumber, portNumber);
        nSoftwareSerialInstances++;
        nThisInstance = nSoftwareSerialInstances;
        softwareSerialInstanceToListen = nThisInstance;
    }

    void begin() {
        this->uartHandle->begin(UARTSENSOR_BAUDRATE);
    }

    void update() {
        if ((millis() - lastTimeThisInstanceChosenMs) > SOFTWARE_SERIAL_INSTANCE_SWITCH_TIMEOUT) {
            while (softwareSerialInstanceToListen != nThisInstance)
                incrementSoftwareSerialInstanceChoice();
#ifdef SERIAL_DEBUG
            Serial.println("SW to");
#endif
        }
        if (softwareSerialInstanceToListen == nThisInstance) {
            lastTimeThisInstanceChosenMs = millis();
            uartHandle->listen();
            while (this->uartHandle->available()) {
                uint8_t rcvByte = this->uartHandle->read();
                if (uartColorSensorBufferPointer == UARTCOLORSENSOR_PACKET_SIZE) {
                    if (rcvByte == UARTCOLORSENSOR_TERMINATOR) {
                        uartColorSensorBufferPointer = 0;
                        incrementSoftwareSerialInstanceChoice();
                        dataOutput.r = dataBuffer.r;
                        dataOutput.g = dataBuffer.g;
                        dataOutput.b = dataBuffer.b;
                    } else {
#ifdef SERIAL_DEBUG
                        Serial.println("SW err");
#endif
                    }
                } else if (uartColorSensorBufferPointer < UARTCOLORSENSOR_PACKET_SIZE) {
                    *((uint8_t *) (&(this->dataBuffer)) + uartColorSensorBufferPointer) = rcvByte;
                    uartColorSensorBufferPointer++;
                }
            }
        }
    }
};


#endif //ARKSTONE_UNIVERSAL_UARTCOLORSENSOR_H
