#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>  
#include "EthernetScannerSDK.h"


#define BUFFER_SIZE 3200

int main() {
    char ip[] = "192.168.100.1";
    char port[] = "32001";

    std::cout << "Connecting to weCat3D sensor at " << ip << ":" << port << "...\n";
    void* handle = EthernetScanner_Connect(ip, port, 1000);

    if (handle == NULL) {
        std::cout << "Connection failed!\n";
        return -1;
    }
    std::cout << "Connection established successfully!\n";

    // Send StartAcquisition command
    const char* startCommand = "StartAcquisition\n";
    int result = EthernetScanner_WriteData(handle, (char*)startCommand, strlen(startCommand));
    if (result != 0) {
        std::cout << "Warning: Failed to send StartAcquisition command, error code: " << result << "\n";
    } else {
        std::cout << "StartAcquisition command sent successfully.\n";
    }

    usleep(200000); 

    // Check connection status
    int status = 0;
    EthernetScanner_GetConnectStatus(handle, &status);
    std::cout << "Connection Status after StartAcquisition: " << status << '\n';

    if (status == 3) {  // Connected
        printf("Successfully connected to sensor.\n");

        double xBuffer[BUFFER_SIZE] = {0};
        double zBuffer[BUFFER_SIZE] = {0};
        int intensityBuffer[BUFFER_SIZE] = {0};
        int signalWidthBuffer[BUFFER_SIZE] = {0};
        unsigned int encoder = 0;
        unsigned char usrIO = 0;
        int pictureCount = 0;

        int numPoints = EthernetScanner_GetXZIExtended(
            handle,
            xBuffer,
            zBuffer,
            intensityBuffer,
            signalWidthBuffer,
            BUFFER_SIZE,
            &encoder,
            &usrIO,
            1000, // timeout
            NULL,
            0,
            &pictureCount
        );

        if (numPoints > 0) {
            std::cout << "Received profile with " << numPoints << " points.\n";
            for (int i = 0; i < std::min(10, numPoints); ++i) {
                std::cout << "Point " << i << ": X = " << xBuffer[i] << " mm, Z = " << zBuffer[i] << " mm\n";
            }
        } else {
            std::cout << "No valid profile received or error code: " << numPoints << "\n";
        }
    } else {
        printf("Failed to fully connect. Status code: %d\n", status);
    }

    // Disconnect
    handle = EthernetScanner_Disconnect(handle);
    if (handle == NULL) {
        printf("Disconnected successfully.\n");
    } else {
        printf("Failed to disconnect.\n");
    }

    return 0;
}
