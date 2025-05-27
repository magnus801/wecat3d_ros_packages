
#include <iostream>
#include <fstream>
#include <cstring>
#include <signal.h>
#include <unistd.h>
#include "EthernetScannerSDK.h"

#define WIDTH_X 1280
#define HEIGHT_Z 1
#define BUFFER_SIZE (WIDTH_X * HEIGHT_Z)

volatile bool keepRunning = true;

void handleSigint(int sig) {
    printf("\nCtrl+C received. Stopping...\n");
    keepRunning = false;
}

void sendCommand(void* handle, const char* cmd) {
    int result = EthernetScanner_WriteData(handle, (char*)cmd, strlen(cmd));
    printf(" (code %d)to send: %s", result, cmd);
}

int main() {
    signal(SIGINT, handleSigint);

    char ip[] = "192.168.100.1";
    char port[] = "32001";

    printf("Connecting to weCat3D at %s:%s...\n", ip, port);
    void* handle = EthernetScanner_Connect(ip, port, 1000);
    if (!handle) {
        printf("Connection failed.\n");
        return -1;
    }
    printf("Connected.\n");

    usleep(200000);
    int status = 0;
    EthernetScanner_GetConnectStatus(handle, &status);
    if (status != 3) {
        printf("Sensor not fully ready. Status: %d\n", status);
        EthernetScanner_Disconnect(handle);
        return -1;
    }

    // Configure internal trigger
    sendCommand(handle, "SetTriggerSource=0\r");
    sendCommand(handle, "SetAcquisitionLineTime=200000\r");  // 5 Hz
    sendCommand(handle, "SetRangeImageNrProfiles=1\r");
    sendCommand(handle, "SetAutoExposureTimeMin=100\r");
    sendCommand(handle, "SetAutoExposureTimeMax=3000\r");
    sendCommand(handle, "SetAutoExposureIntensityRangeMin=300\r");
    sendCommand(handle, "SetAutoExposureIntensityRangeMax=400\r");
    sendCommand(handle, "SetROI1WidthX=1280\r");
    sendCommand(handle, "SetROI1HeightZ=1024\r");

    // Stop any running acquisition
    sendCommand(handle, "SetAcquisitionStop\r");
    usleep(200000);

    // Initialize and enable linearization
    sendCommand(handle, "SetInitializeAcquisition\r");
    sendCommand(handle, "SetLinearizationMode=1\r");
    EthernetScanner_ResetDllFiFo(handle);
    sendCommand(handle, "SetAcquisitionStart\r");

    printf("\nWaiting for profile...\n");

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
    printf("Disconnected.\n");
    return 0;
}

