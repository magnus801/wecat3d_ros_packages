#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>   // Required for strlen
#include <unistd.h>   // For usleep
#include "EthernetScannerSDK.h"


#define SLEEP_MS(ms) usleep((ms) * 1000)
#define PRINT_SEND_RESULT(cmd, res) \
    if (res != 0) { printf("❌ Failed to send: %s (code %d)\n", cmd, res); } \
    else { printf("✅ Sent: %s", cmd); }

void sendCommand(void* handle, const char* cmd) {
    int result = EthernetScanner_WriteData(handle, (char*)cmd, strlen(cmd));
    PRINT_SEND_RESULT(cmd, result);
}

void readProperty(void* handle, const char* name) {
    char buffer[ETHERNETSCANNER_BUFFERSIZEMAX] = {0};
    int result = EthernetScanner_ReadData(handle, (char*)name, buffer, ETHERNETSCANNER_BUFFERSIZEMAX, 0);
    if (result == 0)
        printf("📋 %s = %s\n", name, buffer);
    else
        printf("⚠️  Failed to read %s (code %d)\n", name, result);
}

int main() {
    char ip[] = "192.168.100.1";
    char port[] = "32001";

    printf(" Connecting to weCat3D at %s:%s...\n", ip, port);
    void* handle = EthernetScanner_Connect(ip, port, 1000);
    if (handle == NULL) {
        printf(" Connection failed.\n");
        return -1;
    }
    printf(" Connection initiated.\n");

    // Wait for sensor to be ready
    SLEEP_MS(200);

    int status = 0;
    EthernetScanner_GetConnectStatus(handle, &status);
    printf(" Connection status: %d\n", status);
    if (status != 3) {
        printf(" Sensor not fully ready (expected 3).\n");
        EthernetScanner_Disconnect(handle);
        return -1;
    }

    // Send setup commands
    sendCommand(handle, "SetTriggerSource=2\r");
    sendCommand(handle, "SetTriggerEncoderStep=5\r");
    sendCommand(handle, "SetEncoderTriggerFunction=1\r");
    sendCommand(handle, "ResetEncoder\r");
    sendCommand(handle, "SetRangeImageNrProfiles=100\r");
    sendCommand(handle, "SetROI1WidthX=1280\r");
    sendCommand(handle, "SetROI1HeightZ=1024\r");

    // Reset FIFO
    if (EthernetScanner_ResetDllFiFo(handle) == 0)
        printf("  DLL FIFO reset successful.\n");
    else
        printf(" DLL FIFO reset failed.\n");

    // Start acquisition
    sendCommand(handle, "StartAcquisition\r");

    // Wait for settings to apply
    SLEEP_MS(200);

    // Read back and verify properties
    printf("\n🔍 Verifying parameters from sensor:\n");
    readProperty(handle, "TriggerSource");
    readProperty(handle, "TriggerEncoderStep");
    readProperty(handle, "EncoderTriggerFunction");
    readProperty(handle, "RangeImageNrProfiles");
    readProperty(handle, "ROI1WidthX");
    readProperty(handle, "ROI1HeightZ");

    // Wait before closing
    SLEEP_MS(2000);

    // Disconnect
    handle = EthernetScanner_Disconnect(handle);
    if (handle == NULL) {
        printf("\n🔒 Disconnected successfully.\n");
    } else {
        printf("\n⚠️  Failed to disconnect.\n");
    }

    return 0;
}

