#include "wecat3d_profile_publisher/EthernetScannerDriver.hpp"

// Global variables
volatile bool keep_running = true;
volatile sig_atomic_t stop = false;

void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\nInterrupted by user. Stopping sensor..." << std::endl;
        stop = true;
    }
}

// UserIOState implementation
UserIOState::UserIOState(int ea1, int ea2, int ea3, int ea4, int ttlA, int ttlB, int ttlC)
    : EA1(ea1), EA2(ea2), EA3(ea3), EA4(ea4), TTLEncA(ttlA), TTLEncB(ttlB), TTLEncC(ttlC) {}

// ScannedProfile implementation
ScannedProfile::ScannedProfile() = default;

ScannedProfile::ScannedProfile(int x, int z)
    : roiWidthX(x, 0), roiHeightZ(z, 0), intensity(1200, 0), signalWidth(1200, 0) {}

ScannedProfile::ScannedProfile(ScannedProfile&& other) noexcept
    : roiWidthX(std::move(other.roiWidthX)),
      roiHeightZ(std::move(other.roiHeightZ)),
      intensity(std::move(other.intensity)),
      signalWidth(std::move(other.signalWidth)),
      encoderValue(other.encoderValue),
      userIOState(std::move(other.userIOState)),
      pictureCounter(other.pictureCounter),
      scannedPoints(other.scannedPoints) {}

ScannedProfile& ScannedProfile::operator=(ScannedProfile&& other) noexcept {
    if (this != &other) {
        roiWidthX = std::move(other.roiWidthX);
        roiHeightZ = std::move(other.roiHeightZ);
        intensity = std::move(other.intensity);
        signalWidth = std::move(other.signalWidth);
        encoderValue = other.encoderValue;
        userIOState = std::move(other.userIOState);
        pictureCounter = other.pictureCounter;
        scannedPoints = other.scannedPoints;
    }
    return *this;
}

// Exception class implementation
SensorException::SensorException(const std::string& message)
    : std::runtime_error(message) {}

// Function pointer variables
EthernetScanner_GetConnectStatus_ptr p_EthernetScanner_GetConnectStatus = nullptr;
EthernetScanner_GetXZIExtended_ptr p_EthernetScanner_GetXZIExtended = nullptr;
EthernetScanner_GetImage_ptr p_EthernetScanner_GetImage = nullptr;
EthernetScanner_Connect_ptr p_EthernetScanner_Connect = nullptr;
EthernetScanner_Disconnect_ptr p_EthernetScanner_Disconnect = nullptr;
EthernetScanner_GetDllFiFoState_ptr p_EthernetScanner_GetDllFiFoState = nullptr;
EthernetScanner_ResetDllFiFo_ptr p_EthernetScanner_ResetDllFiFo = nullptr;
EthernetScanner_GetVersion_ptr p_EthernetScanner_GetVersion = nullptr;
EthernetScanner_WriteData_ptr p_EthernetScanner_WriteData = nullptr;
EthernetScanner_ReadData_ptr p_EthernetScanner_ReadData = nullptr;

// Library handle
#ifdef _WIN32
HMODULE libHandle = nullptr;
#else
void* libHandle = nullptr;
#endif

// Library loading/unloading
bool loadEthernetScannerLibrary() {
#ifdef _WIN32
    libHandle = LoadLibrary("EthernetScanner.dll");
#else
    const char* library_paths[] = {
        "./libEthernetScanner.so",
        "libEthernetScanner.so"
    };
    for (const char* path : library_paths) {
        libHandle = dlopen(path, RTLD_LAZY);
        if (libHandle) {
            std::cout << "Successfully loaded library from: " << path << std::endl;
            break;
        }
    }
#endif
    if (!libHandle) {
        std::cerr << "Failed to load EthernetScanner library!" << std::endl;
        std::cerr << "Error: " << dlerror() << std::endl;
        return false;
    }
#ifdef _WIN32
    #define LOAD_SYMBOL(name) (name = (name##_ptr)GetProcAddress(libHandle, #name))
#else
    #define LOAD_SYMBOL(name) (p_##name = (name##_ptr)dlsym(libHandle, #name))
#endif
    if (!LOAD_SYMBOL(EthernetScanner_Connect)) {
        std::cerr << "Failed to load EthernetScanner_Connect: " << dlerror() << std::endl;
        return false;
    }
    if (!LOAD_SYMBOL(EthernetScanner_Disconnect)) {
        std::cerr << "Failed to load EthernetScanner_Disconnect: " << dlerror() << std::endl;
        return false;
    }
    if (!LOAD_SYMBOL(EthernetScanner_GetConnectStatus)) {
        std::cerr << "Failed to load EthernetScanner_GetConnectStatus: " << dlerror() << std::endl;
        return false;
    }
    if (!LOAD_SYMBOL(EthernetScanner_GetXZIExtended)) {
        std::cerr << "Failed to load EthernetScanner_GetXZIExtended: " << dlerror() << std::endl;
        return false;
    }
    if (!LOAD_SYMBOL(EthernetScanner_GetImage)) {
        std::cerr << "Failed to load EthernetScanner_GetImage: " << dlerror() << std::endl;
        return false;
    }
    if (!LOAD_SYMBOL(EthernetScanner_GetDllFiFoState)) {
        std::cerr << "Failed to load EthernetScanner_GetDllFiFoState: " << dlerror() << std::endl;
        return false;
    }
    if (!LOAD_SYMBOL(EthernetScanner_ResetDllFiFo)) {
        std::cerr << "Failed to load EthernetScanner_ResetDllFiFo: " << dlerror() << std::endl;
        return false;
    }
    if (!LOAD_SYMBOL(EthernetScanner_GetVersion)) {
        std::cerr << "Failed to load EthernetScanner_GetVersion: " << dlerror() << std::endl;
        return false;
    }
    if (!LOAD_SYMBOL(EthernetScanner_WriteData)) {
        std::cerr << "Failed to load EthernetScanner_WriteData: " << dlerror() << std::endl;
        return false;
    }
    if (!LOAD_SYMBOL(EthernetScanner_ReadData)) {
        std::cerr << "Failed to load EthernetScanner_ReadData: " << dlerror() << std::endl;
        return false;
    }
    return true;
}

void unloadEthernetScannerLibrary() {
    if (libHandle) {
#ifdef _WIN32
        FreeLibrary(libHandle);
#else
        dlclose(libHandle);
#endif
        libHandle = nullptr;
    }
}

// Sensor class implementation
Sensor::Sensor(const std::string& ipAddress, int portNumber)
    : ip(ipAddress), port(std::to_string(portNumber)),
      handle(nullptr), status(0), rawBufferSize(0) {
    read_buf = std::make_unique<char[]>(read_buf_size);
    std::memset(read_buf.get(), 0, read_buf_size);
}

void Sensor::connect(int timeout) {
    if (!p_EthernetScanner_Connect) {
        throw std::runtime_error("EthernetScanner_Connect function not loaded!");
    }
    std::vector<char> ip_copy(ip.begin(), ip.end());
    ip_copy.push_back('\0');
    std::vector<char> port_copy(port.begin(), port.end());
    port_copy.push_back('\0');
    std::cout << "Calling EthernetScanner_Connect with IP: " << ip << ", Port: " << port << ", Timeout: " << timeout << std::endl;
    handle = p_EthernetScanner_Connect(ip_copy.data(), port_copy.data(), timeout);
    if (!handle) {
        std::cout << "Initial connection attempt returned null handle" << std::endl;
    } else {
        std::cout << "Got handle from EthernetScanner_Connect: " << handle << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    auto connectionPollingStart = std::chrono::steady_clock::now();
    while (!handle && std::chrono::steady_clock::now() - connectionPollingStart < std::chrono::milliseconds(timeout)) {
    }
    int connectionStatus = get_connect_status();
    if (connectionStatus != SENSOR_CONNECTED) {
        throw std::runtime_error("Failed to connect with the sensor; Connection status: " + std::to_string(connectionStatus));
    }
    std::cout << "Sensor has been connected successfully. Status: " << connectionStatus << "\n";
    allocate_memory();
}

void Sensor::disconnect() {
    handle = p_EthernetScanner_Disconnect(handle);
    if (handle) {
        throw std::runtime_error("Failed to disconnect the sensor.");
    }
    deallocate_memory();
}

int Sensor::get_connect_status() {
    if (!p_EthernetScanner_GetConnectStatus) {
        return SENSOR_DISCONNECTED;
    }
    if (!handle) {
        return SENSOR_DISCONNECTED;
    }
    p_EthernetScanner_GetConnectStatus(handle, &status);
    int result = (status & SENSOR_CONNECTED) ? SENSOR_CONNECTED : SENSOR_DISCONNECTED;
    return result;
}

void Sensor::allocate_memory() {
    roiWidthX = std::make_unique<double[]>(SENSOR_BUFFERSIZEMAX);
    roiHeightZ = std::make_unique<double[]>(SENSOR_BUFFERSIZEMAX);
    intensity = std::make_unique<int[]>(SENSOR_BUFFERSIZEMAX);
    signalWidth = std::make_unique<int[]>(SENSOR_BUFFERSIZEMAX);
    encoderValue = std::make_unique<uint32_t>(0);
    userIOState = std::make_unique<uint32_t>(0);
    pictureCounter = std::make_unique<int>(0);
    rawBuffer = std::make_unique<char[]>(SENSOR_BUFFERSIZEMAX);
    rawBufferSize = SENSOR_BUFFERSIZEMAX;
    width = std::make_unique<uint32_t[]>(SENSOR_BUFFERSIZEMAX);
    height = std::make_unique<uint32_t[]>(SENSOR_BUFFERSIZEMAX);
    offsetX = std::make_unique<uint32_t[]>(SENSOR_BUFFERSIZEMAX);
    offsetY = std::make_unique<uint32_t[]>(SENSOR_BUFFERSIZEMAX);
    stepX = std::make_unique<uint32_t[]>(SENSOR_BUFFERSIZEMAX);
    stepY = std::make_unique<uint32_t[]>(SENSOR_BUFFERSIZEMAX);
    std::memset(roiWidthX.get(), 0, SENSOR_BUFFERSIZEMAX * sizeof(double));
    std::memset(roiHeightZ.get(), 0, SENSOR_BUFFERSIZEMAX * sizeof(double));
    std::memset(intensity.get(), 0, SENSOR_BUFFERSIZEMAX * sizeof(int));
    std::memset(signalWidth.get(), 0, SENSOR_BUFFERSIZEMAX * sizeof(int));
    std::memset(rawBuffer.get(), 0, SENSOR_BUFFERSIZEMAX);
    std::memset(width.get(), 0, SENSOR_BUFFERSIZEMAX * sizeof(uint32_t));
    std::memset(height.get(), 0, SENSOR_BUFFERSIZEMAX * sizeof(uint32_t));
    std::memset(offsetX.get(), 0, SENSOR_BUFFERSIZEMAX * sizeof(uint32_t));
    std::memset(offsetY.get(), 0, SENSOR_BUFFERSIZEMAX * sizeof(uint32_t));
    std::memset(stepX.get(), 0, SENSOR_BUFFERSIZEMAX * sizeof(uint32_t));
    std::memset(stepY.get(), 0, SENSOR_BUFFERSIZEMAX * sizeof(uint32_t));
}

void Sensor::deallocate_memory() {
    roiWidthX.reset();
    roiHeightZ.reset();
    intensity.reset();
    signalWidth.reset();
    encoderValue.reset();
    userIOState.reset();
    pictureCounter.reset();
    rawBuffer.reset();
    width.reset();
    height.reset();
    offsetX.reset();
    offsetY.reset();
    stepX.reset();
    stepY.reset();
    rawBufferSize = 0;
}

CameraImage Sensor::get_camera_image(int timeout) {
    std::cout << "Reading camera image ...\n";
    int response = p_EthernetScanner_GetImage(
        handle,
        rawBuffer.get(),
        rawBufferSize,
        width.get(),
        height.get(),
        offsetX.get(),
        offsetY.get(),
        stepX.get(),
        stepY.get(),
        static_cast<uint32_t>(timeout)
    );
    if (response < 0) {
        throw SensorException("Failed to get camera image. Error code: " + std::to_string(response));
    }
    std::cout << "Camera image received successfully\n";
    uint32_t imgResolution = width[0] * height[0];
    std::vector<uint8_t> imgData(rawBuffer.get(), rawBuffer.get() + imgResolution);
    CameraImage img;
    img.rawImageData = std::move(imgData);
    img.imgWidth = width[0];
    img.imgHeight = height[0];
    img.imgOffsetX = offsetX[0];
    img.imgOffsetY = offsetY[0];
    img.imgStepX = stepX[0];
    img.imgStepY = stepY[0];
    return img;
}

void Sensor::get_scanned_profile(ScannedProfile& profile, int timeout) {
    unsigned char* ucBufferRaw = nullptr;
    int iBufferRaw = 0;
    int response = p_EthernetScanner_GetXZIExtended(
        handle,
        roiWidthX.get(),
        roiHeightZ.get(),
        intensity.get(),
        signalWidth.get(),
        rawBufferSize,
        encoderValue.get(),
        (unsigned char*)userIOState.get(),
        timeout,
        ucBufferRaw,
        iBufferRaw,
        pictureCounter.get()
    );
    if (response < 0) {
        throw SensorException("Failed to get scanned profile. Error code: " + std::to_string(response));
    }
    profile.encoderValue = *encoderValue;
    profile.userIOState = UserIOState(
        ((*userIOState) & 0b1),
        ((*userIOState) & 0b10) >> 1,
        ((*userIOState) & 0b100) >> 2,
        ((*userIOState) & 0b1000) >> 3,
        ((*userIOState) & 0b10000) >> 4,
        ((*userIOState) & 0b100000) >> 5,
        ((*userIOState) & 0b1000000) >> 6
    );
    for (int i = 0; i < response; ++i) {
        profile.roiWidthX[i] = roiWidthX[i];
        profile.roiHeightZ[i] = roiHeightZ[i];
    }
    profile.pictureCounter = *pictureCounter;
    profile.scannedPoints = response;
}

std::string Sensor::get_dll_version() {
    int response = p_EthernetScanner_GetVersion(reinterpret_cast<unsigned char*>(read_buf.get()), read_buf_size);
    if (response < 0) {
        throw SensorException("Failed to get DLL version. Error code: " + std::to_string(response));
    }
    return std::string(read_buf.get());
}

std::string Sensor::read_data(const std::string& command, int cacheTime) {
    std::vector<char> cmdBuffer(command.begin(), command.end());
    cmdBuffer.push_back('\0');
    int response = p_EthernetScanner_ReadData(
        handle,
        cmdBuffer.data(),
        read_buf.get(),
        read_buf_size,
        cacheTime
    );
    if (response == SENSOR_READDATAOK) {
        std::cout << "Sent Read Data command " << command << " -> Response: " << read_buf.get() << " (Sensor_READDATAOK)\n";
    } else {
        throw SensorException("Failed to read data. Error code: " + std::to_string(response));
    }
    return std::string(read_buf.get());
}

int Sensor::write_data(const std::string& command) {
    std::vector<char> cmdBuffer(command.begin(), command.end());
    cmdBuffer.push_back('\0');
    int response = p_EthernetScanner_WriteData(
        handle,
        cmdBuffer.data(),
        static_cast<int>(command.size())
    );
    int responseExpected = static_cast<int>(command.size());
    if (response == responseExpected) {
        std::cout << "Sent Write Data command " << command << " -> Response: " << response << " (len(command))\n";
    } else if (response > 0 && response < responseExpected) {
        std::cerr << "Warning: Sent Write Data command " << command << " -> Response: " << response << "; Expected: " << responseExpected << " (len(command))\n";
    } else {
        throw SensorException("Failed to write data. Error code: " + std::to_string(response));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return response;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    signal(SIGINT, signal_handler);
    auto node = rclcpp::Node::make_shared("wecat3d_runtime_node");
    auto pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/wecat3d/pointcloud", 10);
    std::ofstream pcd_file("merged_encoder_output.pcd", std::ios::out | std::ios::trunc);
    if (!pcd_file.is_open()) {
        std::cerr << "Failed to open PCD file for writing!" << std::endl;
        return 1;
    }
    std::streampos header_start = pcd_file.tellp();
    pcd_file << "# .PCD v0.7 - Point Cloud Data file\n";
    pcd_file << "VERSION 0.7\n";
    pcd_file << "FIELDS x y z\n";
    pcd_file << "SIZE 4 4 4\n";
    pcd_file << "TYPE F F F\n";
    pcd_file << "COUNT 1 1 1\n";
    pcd_file << "WIDTH 0000000000\n";
    pcd_file << "HEIGHT 1\n";
    pcd_file << "VIEWPOINT 0 0 0 1 0 0 0\n";
    pcd_file << "POINTS 0000000000\n";
    pcd_file << "DATA ascii\n";
    std::streampos data_start = pcd_file.tellp();
    uint64_t total_points = 0;
    if (!loadEthernetScannerLibrary()) {
        std::cerr << "Failed to load EthernetScanner library!" << std::endl;
        pcd_file.close();
        return 1;
    }
    try {
        Sensor sensor("192.168.100.1", 32001);
        sensor.connect(0);
        sensor.write_data("SetHeartbeat=1000");
        sensor.write_data("SetExposureTime=750");
        sensor.write_data("SetTriggerSource=2");
        sensor.write_data("SetTriggerEncoderStep=20");
        sensor.write_data("SetEncoderTriggerFunction=2");
        sensor.write_data("ResetEncoder");
        sensor.write_data("SetRangeImageNrProfiles=1");
        sensor.write_data("SetAcquisitionStart");
        int scanNo = 0;
        int64_t last_encoder = std::numeric_limits<int64_t>::min();
        sensor_msgs::msg::PointCloud2 msg;
        sensor_msgs::PointCloud2Modifier mod(msg);
        mod.setPointCloud2FieldsByString(1, "xyz");
        msg.header.frame_id = "map";
        msg.height = 1;
        msg.is_dense = false;
        msg.is_bigendian = false;
        const float scale_factor = 1.0f / 1000.0f;
        std::signal(SIGINT, signal_handler);
        ScannedProfile scan(1280, 1024);
        while (!stop) {
            try {
                sensor.get_scanned_profile(scan, 1000);
                scanNo++;
                const std::vector<float>& x_values = scan.roiWidthX;
                const std::vector<float>& z_values = scan.roiHeightZ;
                uint32_t encoder_unsigned = scan.encoderValue;
                int64_t encoder = static_cast<int64_t>(encoder_unsigned);
                if (encoder_unsigned >= static_cast<uint32_t>(1ULL << 31)) {
                    encoder -= static_cast<int64_t>(1ULL << 32);
                }
                if (last_encoder != std::numeric_limits<int64_t>::min()) {
                    int64_t delta_encoder = encoder - last_encoder;
                    double delta_microns = delta_encoder * (ENC_SCALE_MM * 1000.0);
                }
                if (last_encoder == std::numeric_limits<int64_t>::min() || encoder != last_encoder) {
                    double y_value = encoder * ENC_SCALE_MM;
                    for (size_t i = 0; i < x_values.size(); ++i) {
                        pcd_file << std::fixed << std::setprecision(6)
                                << x_values[i] << " "
                                << y_value << " "
                                << z_values[i] << "\n";
                    }
                    pcd_file.flush();
                    total_points += x_values.size();
                    msg.header.stamp = node->now();
                    if (msg.width != x_values.size()) {
                        msg.width = x_values.size();
                        mod.resize(x_values.size());
                    }
                    sensor_msgs::PointCloud2Iterator<float> it_x(msg, "x");
                    sensor_msgs::PointCloud2Iterator<float> it_y(msg, "y");
                    sensor_msgs::PointCloud2Iterator<float> it_z(msg, "z");
                    float y_scaled = y_value * scale_factor;
                    for (size_t i = 0; i < x_values.size(); ++i, ++it_x, ++it_y, ++it_z) {
                        *it_x = x_values[i] * scale_factor;
                        *it_y = y_scaled;
                        *it_z = z_values[i] * scale_factor;
                    }
                    pub->publish(msg);
                    if (scanNo % 500 == 0) {
                        std::cout << "Processed " << scanNo << " scans, total points: " << total_points << std::endl;
                    }
                }
                last_encoder = encoder;
            } catch (const SensorException& e) {
                if (std::string(e.what()).find("-1") != std::string::npos) {
                    continue;
                } else {
                    throw;
                }
            }
        }
        std::streampos current_pos = pcd_file.tellp();
        pcd_file.seekp(header_start);
        pcd_file << "# .PCD v0.7 - Point Cloud Data file\n";
        pcd_file << "VERSION 0.7\n";
        pcd_file << "FIELDS x y z\n";
        pcd_file << "SIZE 4 4 4\n";
        pcd_file << "TYPE F F F\n";
        pcd_file << "COUNT 1 1 1\n";
        pcd_file << "WIDTH " << std::setw(10) << std::setfill('0') << total_points << "\n";
        pcd_file << "HEIGHT 1\n";
        pcd_file << "VIEWPOINT 0 0 0 1 0 0 0\n";
        pcd_file << "POINTS " << std::setw(10) << std::setfill('0') << total_points << "\n";
        pcd_file << "DATA ascii\n";
        pcd_file.close();
        std::cout << "Final PCD saved with " << total_points << " points" << std::endl;
        std::cout << "Getting connection status..." << std::endl;
        int status = sensor.get_connect_status();
        if (status == SENSOR_CONNECTED) {
            std::cout << "Sensor is confirmed to be connected." << std::endl;
        } else {
            std::cout << "Sensor is NOT connected after attempt." << std::endl;
        }
        sensor.write_data("SetAcquisitionStop");
        sensor.write_data("SetResetSettings");
        sensor.disconnect();
        rclcpp::shutdown();
    } catch (const std::exception& e) {
        std::cerr << "Exception occurred: " << e.what() << std::endl;
        pcd_file.close();
        unloadEthernetScannerLibrary();
        return 1;
    }
    unloadEthernetScannerLibrary();
    return 0;
}
