#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>  // Required for strlen
#include "EthernetScannerSDK.h"
#include <unistd.h>

int main() {
    // Connect to sensor
    char ip[] = "192.168.100.1";
    char port[] = "32001";
    void* handle = EthernetScanner_Connect(ip, port, 1000);
    if (handle == NULL) {
        printf("Connection failed.\n");
        return -1;
    }
    printf("Connection initiated.\n");

    // Send StartAcquisition command
    const char* startCommand = "StartAcquisition\n";
    int result = EthernetScanner_WriteData(handle, (char*)startCommand, strlen(startCommand));
    if (result != 0) {
        printf("Warning: Failed to send StartAcquisition command, result: %d\n", result);
    } else {
        printf("StartAcquisition command sent successfully.\n");
    }

    // Small delay to allow sensor to process
    usleep(200000); // 200ms

    // Check connection status
    int status = 0;
    EthernetScanner_GetConnectStatus(handle, &status);
    std::cout << "Connection status after StartAcquisition: " << status << '\n';
    if (status == 3) {  // ETHERNETSCANNER_TCPSCANNERCONNECTED
        printf("Successfully connected to sensor.\n");
    } else {
        printf("Failed to connect. Status code: %d\n", status);
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

