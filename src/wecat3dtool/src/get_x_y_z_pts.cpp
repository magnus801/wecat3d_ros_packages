#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <chrono>
#include <csignal>
#include <cstdint>
#include "EthernetScannerSDK.h"


#define BUFFER_SIZE 3200

volatile bool running = true;

void handleSigint(int) {
    std::cout << "\nCtrl+C detected. Exiting acquisition loop...\n";
    running = false;
}

void sendCommand(void* handle, const char* command) {
    int res = EthernetScanner_WriteData(handle, (char*)command, strlen(command));
    if (res != 0) {
        std::cerr << " Failed to send command: " << command << " | Error code: " << res << "\n";
    } else {
        std::cout << " Command sent: " << command;
    }
}

int main() {
    signal(SIGINT, handleSigint);

    char ip[] = "192.168.100.1";
    char port[] = "32001";

    std::cout << "Connecting to weCat3D sensor at " << ip << ":" << port << "...\n";
    void* handle = EthernetScanner_Connect(ip, port, 0);
    if (handle == NULL) {
        std::cerr << "Connection failed!\n";
        return -1;
    }
    std::cout << " Connection established successfully!\n";

    // Wait for full connection
    int status = 0, retries = 0;
    while (status != 3 && retries++ < 10) {
        EthernetScanner_GetConnectStatus(handle, &status);
        usleep(200000);
    }
    if (status != 3) {
        std::cerr << " Failed to fully connect. Status code: " << status << "\n";
        EthernetScanner_Disconnect(handle);
        return -1;
    }

    std::cout << " Sensor fully connected. Sending configuration commands...\n";
    sendCommand(handle, "SetTriggerSource=2\r");
    sendCommand(handle, "SetTriggerEncoderStep=10\r");
    sendCommand(handle, "SetEncoderTriggerFunction=1\r");
    sendCommand(handle, "ResetEncoder\r\n");
    sendCommand(handle, "StartAcquisition\r\n");

    std::cout << "Acquisition started. Press Ctrl+C to stop...\n";

    // Motion detection setup
    static int32_t lastEncoder = -1;
    static int stillCounter = 0;
    const int stillThreshold = 5;
    const int yStepThreshold = 1;

    while (running) {
        double xBuffer[BUFFER_SIZE] = {0};
        double zBuffer[BUFFER_SIZE] = {0};
        int intensityBuffer[BUFFER_SIZE] = {0};
        int signalWidthBuffer[BUFFER_SIZE] = {0};
        int32_t encoder = 0;
        unsigned char usrIO = 0;
        int pictureCount = 0;

        int numPoints = EthernetScanner_GetXZIExtended(
            handle,
            xBuffer,
            zBuffer,
            intensityBuffer,
            signalWidthBuffer,
            BUFFER_SIZE,
            (unsigned int*)&encoder,
            &usrIO,
            1000,
            NULL,
            0,
            &pictureCount
        );

        // Check for invalid profile
        // bool validProfile = false;
        // for (int i = 0; i < numPoints; ++i) {
        //     if (xBuffer[i] != 0 || zBuffer[i] != 0) {
        //         validProfile = true;
        //         break;
        //     }
        // }
        // if (!validProfile) {
        //     std::cout << " Empty profile skipped (all zero values).\n";
        //     continue;
        // }

        // // Check motion
        // int yDelta = abs(encoder - lastEncoder);
        // if (yDelta <= yStepThreshold) {
        //     stillCounter++;
        // } else {
        //     stillCounter = 0;
        // }
        // lastEncoder = encoder;

        // if (stillCounter >= stillThreshold) {
        //     std::cout << " Motion stopped. Skipping further profiles...\n";
        //     usleep(500000);  // pause 0.5 sec while stationary
        //     continue;
        // }

        // Output valid profile
        std::cout << "\n🔹 Profile (" << numPoints << " points) | Encoder Y: " << encoder << "\n";
        for (int i = 0; i < std::min(10, numPoints); ++i) {
            std::cout << "X: " << xBuffer[i] << " mm, Y: " << encoder << ", Z: " << zBuffer[i] << " mm\n";
        }

        usleep(100000);  // 100 ms delay
    }

    std::cout << "\n Stopping acquisition and disconnecting...\n";
    handle = EthernetScanner_Disconnect(handle);
    if (handle == NULL) {
        std::cout << " Disconnected successfully.\n";
    } else {
        std::cerr << " Failed to disconnect properly.\n";
    }

    return 0;
}
