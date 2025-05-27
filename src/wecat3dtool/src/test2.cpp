#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include "EthernetScannerSDK.h"
#include "EthernetScannerSDKDefine.h"

#define WIDTH_X 1280
#define HEIGHT_Z 100
#define BUFFER_SIZE (WIDTH_X * HEIGHT_Z)

volatile bool keepRunning = true;

void handleSigint(int sig) {
    printf("\n Ctrl+C received. Stopping...\n");
    keepRunning = false;
}

void sendCommand(void* handle, const char* cmd) {
    int result = EthernetScanner_WriteData(handle, (char*)cmd, strlen(cmd));
    printf(" (code %d) to send: %s", result, cmd);
}

void readProperty(void* handle, const char* name) {
    char buffer[ETHERNETSCANNER_BUFFERSIZEMAX] = {0};
    int result = EthernetScanner_ReadData(handle, (char*)name, buffer, ETHERNETSCANNER_BUFFERSIZEMAX, 0);
    if (result == 0)
        printf(" %s = %s\n", name, buffer);
    else
        printf(" Failed to read %s (code %d)\n", name, result);
}

int main() {
    signal(SIGINT, handleSigint);

    char ip[] = "192.168.100.1";
    char port[] = "32001";

    printf(" Connecting to weCat3D at %s:%s...\n", ip, port);
    void* handle = EthernetScanner_Connect(ip, port, 1000);
    if (handle == NULL) {
        printf("  Connection failed.\n");
        return -1;
    }
    printf("  Connection initiated.\n");

    // Wait for sensor readiness (status = 3)
    int status = 0, attempts = 0;
    while (status != 3 && attempts++ < 10) {
        usleep(200000);  // 200 ms
        EthernetScanner_GetConnectStatus(handle, &status);
    }
    printf(" Connection status: %d\n", status);
    if (status != 3) {
        printf("  Sensor not fully ready.\n");
        EthernetScanner_Disconnect(handle);
        return -1;
    }

    // Sensor Configuration
    sendCommand(handle, "SetTriggerSource=2\r");
    sendCommand(handle, "SetTriggerEncoderStep=10\r");
    sendCommand(handle, "SetEncoderTriggerFunction=1\r");
    sendCommand(handle, "ResetEncoder\r");
    sendCommand(handle, "SetRangeImageNrProfiles=10\r");
    sendCommand(handle, "SetROI1WidthX=1280\r");
    sendCommand(handle, "SetROI1HeightZ=1024\r");

    // Proper Setup Sequence
    sendCommand(handle, "SetAcquisitionStop\r");
    usleep(200000);
    sendCommand(handle, "SetLinearizationMode=1\r");
    sendCommand(handle, "SetInitializeAcquisition\r");
    EthernetScanner_ResetDllFiFo(handle);
    sendCommand(handle, "SetAcquisitionStart\r");

    // Read key sensor properties
    printf("\n Verifying parameters from sensor:\n");
    readProperty(handle, "TriggerSource");
    readProperty(handle, "TriggerEncoderStep");
    readProperty(handle, "EncoderTriggerFunction");
    readProperty(handle, "RangeImageNrProfiles");
    readProperty(handle, "ROI1WidthX");
    readProperty(handle, "ROI1HeightZ");

    printf("\n Reading profiles. Press Ctrl+C to stop...\n");

    int32_t lastEncoder = -1;

    while (keepRunning) {
        double xBuffer[BUFFER_SIZE] = {0};
        double zBuffer[BUFFER_SIZE] = {0};
        int intensityBuffer[BUFFER_SIZE] = {0};
        int signalWidthBuffer[BUFFER_SIZE] = {0};
        unsigned int encoder = 0;
        unsigned char usrIO = 0;
        int pictureCount = 0;

        int numPoints = EthernetScanner_GetXZIExtended(
            handle,
            xBuffer, zBuffer,
            intensityBuffer, signalWidthBuffer,
            BUFFER_SIZE,
            &encoder,
            &usrIO,
            1000, nullptr, 0, &pictureCount
        );

        if (numPoints > 0 && encoder != lastEncoder) {
            printf("\n Profile #%d | Encoder: %d | Points: %d\n", pictureCount, encoder, numPoints);
            for (int i = 0; i < numPoints && i < 10; ++i) {
                printf("X: %.2f\tZ: %.2f\tI: %d\n", xBuffer[i], zBuffer[i], intensityBuffer[i]);
            }
            lastEncoder = encoder;
        } else if (encoder == lastEncoder) {
            printf(" Waiting... No new encoder motion detected.\n");
        } else {
            printf(" No valid profile received (code %d)\n", numPoints);
        }

        usleep(100000);  // 100 ms
    }

    handle = EthernetScanner_Disconnect(handle);
    if (handle == NULL) {
        printf(" Disconnected successfully.\n");
    } else {
        printf(" Failed to disconnect.\n");
    }

    return 0;
}

