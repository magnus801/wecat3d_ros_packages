#include <iostream>
#include <fstream>
#include <cstring>
#include <unistd.h>
#include "EthernetScannerSDK.h"
#include "EthernetScannerSDKDefine.h"  // For ETHERNETSCANNER_BUFFERSIZEMAX

#define BUFFER_SIZE 3200

void readProperty(void* handle, const char* propertyName) {
    char resultBuffer[ETHERNETSCANNER_BUFFERSIZEMAX] = {0};

    int res = EthernetScanner_ReadData(
        handle,
        (char*)propertyName,
        resultBuffer,
        ETHERNETSCANNER_BUFFERSIZEMAX,
        0  // XML mode
    );

    if (res == 0) {
        std::cout << "📋 " << propertyName << " = " << resultBuffer << "\n";
    } else {
        std::cerr << "⚠️ Failed to read [" << propertyName << "] | Error Code: " << res << "\n";
    }
}

int main() {
    char ip[] = "192.168.100.1";
    char port[] = "32001";

    std::cout << "Connecting to weCat3D sensor at " << ip << ":" << port << "...\n";
    void* handle = EthernetScanner_Connect(ip, port, 1000);

    if (!handle) {
        std::cerr << "Connection failed!\n";
        return -1;
    }
    std::cout << "Connection established.\n";

    // Trigger config and acquisition
    EthernetScanner_WriteData(handle, (char*)"SetTriggerSource=0\r", strlen("SetTriggerSource=0\r"));
    EthernetScanner_WriteData(handle, (char*)"ResetEncoder\r", strlen("ResetEncoder\r"));
    EthernetScanner_WriteData(handle, (char*)"StartAcquisition\r", strlen("StartAcquisition\r"));
    usleep(200000);

    readProperty(handle, "ROI1WidthX");
    readProperty(handle, "ROI1HeightZ");

    // Buffers
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
        1000,
        nullptr,
        0,
        &pictureCount
    );

    if (numPoints > 0) {
        std::cout << "Received profile with " << numPoints << " points.\n";
        std::ofstream outFile("profile_output.txt");
        if (!outFile.is_open()) {
            std::cerr << "Failed to open output file.\n";
            EthernetScanner_Disconnect(handle);
            return -1;
        }

        for (int i = 0; i < numPoints; ++i) {
            outFile << xBuffer[i] << " " << zBuffer[i] << " " << intensityBuffer[i] << "\n";
        }

        outFile.close();
        std::cout << "Saved to profile_output.txt\n";
    } else {
        std::cerr << "No valid profile received. Code: " << numPoints << "\n";
    }

    EthernetScanner_Disconnect(handle);
    return 0;
}
