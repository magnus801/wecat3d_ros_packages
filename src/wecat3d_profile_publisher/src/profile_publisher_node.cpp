#include <iostream>
#include <csignal>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <thread>
#include <exception>
#include <string>
#include <chrono>
#include <iomanip>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <dlfcn.h>
#include "EthernetScannerSDK.h"

/* ───────── Macros and constants (unchanged) ───────── */
#define WIDTH_X 1280
#define HEIGHT_Z 1024
#define SENSOR_BUFFERSIZEMAX 4202500
#define ENC_SCALE_MM 0.02
#define PCD_FILE "merged_encoder_output.pcd"

#define SENSOR_CONNECTED 3
#define SENSOR_DISCONNECTED 0
#define SENSOR_READDATAOK 0
#define SENSOR_ERROR -1
#define SENSOR_GETINFOSMALLBUFFER -2
#define SENSOR_GETINFONOVALIDINFO -3
#define SENSOR_GETINFOINVALIDXML -4
#define SENSOR_GETXZINONEWSCAN -1
#define SENSOR_GETXZIINVALIDLINDATA -2
#define SENSOR_GETXZIINVALIDBUFFER -3
#define SENSOR_READDATAOK 0
#define SENSOR_READDATASMALLBUFFER -1
#define SENSOR_READDATANOTSUPPORTEDMODE -2
#define SENSOR_READDATAFEATURENOTDEFINED -3
#define SENSOR_READDATANOSCAN -4
#define SENSOR_READDATAFAILED -5
#define SENSOR_WRITEDATAINVALIDSOCKET -1
#define SENSOR_INVALIDHANDLE -1000
#define SENSOR_GETINFOSMALLERBUFFER -2

/* --- global CTRL-C flag -------------------------------------------- */
volatile sig_atomic_t stop_flag = 0;
void signal_handler(int sig) { if (sig == SIGINT) stop_flag = 1; }

/* ───────── helper structs exactly as before ───────── */
struct UserIOState {
    int EA1{}, EA2{}, EA3{}, EA4{};
    int TTLEncA{}, TTLEncB{}, TTLEncC{};
};

struct ScannedProfile {
    std::vector<float> roiWidthX;
    std::vector<float> roiHeightZ;
    std::vector<uint8_t> intensity;
    std::vector<float> signalWidth;
    unsigned int encoderValue{};
    UserIOState userIOState;
    int pictureCounter{};
    int scannedPoints{};
};



/* ───────── Function pointer typedefs ───────── */
using GetConnectStatus_t   = void (*)(void*, int*);
using GetXZIExtended_t     = int  (*)(void*, double*, double*, int*, int*, int,
                                       unsigned int*, unsigned char*, int,
                                       unsigned char*, int, int*);
using GetImage_t           = int  (*)(void*, char*, int, unsigned int*, unsigned int*,
                                       unsigned int*, unsigned int*, unsigned int*,
                                       unsigned int*, unsigned int);
using Connect_t            = void*(*)(char*, char*, int);
using Disconnect_t         = void*(*)(void*);
using GetDllFiFoState_t    = int  (*)(void*);
using ResetDllFiFo_t       = int  (*)(void*);
using GetVersion_t         = int  (*)(unsigned char*, int);
using WriteData_t          = int  (*)(void*, char*, int);
using ReadData_t           = int  (*)(void*, char*, char*, int, int);

/* ───────── global DLL handles (unchanged names) ───────── */
static GetConnectStatus_t   p_GetConnectStatus   = nullptr;
static GetXZIExtended_t     p_GetXZIExtended     = nullptr;
static GetImage_t           p_GetImage           = nullptr;
static Connect_t            p_Connect            = nullptr;
static Disconnect_t         p_Disconnect         = nullptr;
static GetDllFiFoState_t    p_GetDllFiFoState    = nullptr;
static ResetDllFiFo_t       p_ResetDllFiFo       = nullptr;
static GetVersion_t         p_GetVersion         = nullptr;
static WriteData_t          p_WriteData          = nullptr;
static ReadData_t           p_ReadData           = nullptr;

/* ───────────────── 1.  typedefs that match the DLL ───────────────── */
using EthernetScanner_Connect_ptr            = void* (*)(char*, char*, int);
using EthernetScanner_Disconnect_ptr         = void* (*)(void*);
using EthernetScanner_GetConnectStatus_ptr   = void  (*)(void*, int*);
using EthernetScanner_GetXZIExtended_ptr     = int   (*)(void*, double*, double*, int*, int*, int,
                                                         unsigned int*, unsigned char*, int,
                                                         unsigned char*, int, int*);
using EthernetScanner_GetImage_ptr           = int   (*)(void*, char*, int, unsigned int*, unsigned int*,
                                                         unsigned int*, unsigned int*, unsigned int*,
                                                         unsigned int*, unsigned int);
using EthernetScanner_GetDllFiFoState_ptr    = int   (*)(void*);
using EthernetScanner_ResetDllFiFo_ptr       = int   (*)(void*);
using EthernetScanner_GetVersion_ptr         = int   (*)(unsigned char*, int);
using EthernetScanner_WriteData_ptr          = int   (*)(void*, char*, int);
using EthernetScanner_ReadData_ptr           = int   (*)(void*, char*, char*, int, int);

/* ───────────────── 2.  global pointer variables ──────────────────── */
static EthernetScanner_Connect_ptr            p_EthernetScanner_Connect            = nullptr;
static EthernetScanner_Disconnect_ptr         p_EthernetScanner_Disconnect         = nullptr;
static EthernetScanner_GetConnectStatus_ptr   p_EthernetScanner_GetConnectStatus   = nullptr;
static EthernetScanner_GetXZIExtended_ptr     p_EthernetScanner_GetXZIExtended     = nullptr;
static EthernetScanner_GetImage_ptr           p_EthernetScanner_GetImage           = nullptr;
static EthernetScanner_GetDllFiFoState_ptr    p_EthernetScanner_GetDllFiFoState    = nullptr;
static EthernetScanner_ResetDllFiFo_ptr       p_EthernetScanner_ResetDllFiFo       = nullptr;
static EthernetScanner_GetVersion_ptr         p_EthernetScanner_GetVersion         = nullptr;
static EthernetScanner_WriteData_ptr          p_EthernetScanner_WriteData          = nullptr;
static EthernetScanner_ReadData_ptr           p_EthernetScanner_ReadData           = nullptr;

/* ───────────────── 3.  platform-neutral loader macro ─────────────── */
#ifdef _WIN32
  #define LOAD_SYMBOL(name)                                                     \
      (p_##name = (name##_ptr)GetProcAddress(libHandle, #name))
#else
  #define LOAD_SYMBOL(name)                                                     \
      (p_##name = (name##_ptr)dlsym(libHandle, #name))
#endif

/* ───────────────── 4.  libHandle declaration ─────────────────────── */
#ifdef _WIN32
  static HMODULE libHandle = nullptr;
#else
  static void*   libHandle = nullptr;
#endif

void unloadEthernetScannerLibrary()
{
    if (!libHandle) return;
#ifdef _WIN32
    FreeLibrary(libHandle);
#else
    dlclose(libHandle);
#endif
    libHandle = nullptr;
    std::cout << "Library unloaded successfully.\n";
}
/* ───────────────── 5.  load / unload functions ───────────────────── */
bool loadEthernetScannerLibrary()
{
    /* --- open the .so/.dll --- */
#ifdef _WIN32
    libHandle = LoadLibrary("EthernetScanner.dll");
#else
    const char* paths[] = { "./libEthernetScanner.so",
                            "/usr/local/lib/libEthernetScanner.so",
                            "/usr/lib/libEthernetScanner.so",
                            "libEthernetScanner.so" };
    for (const char* p : paths)
        if ((libHandle = dlopen(p, RTLD_LAZY))) {
            std::cout << "Successfully loaded library from: " << p << '\n'; break;
        }
#endif
    if (!libHandle) {
        std::cerr << "Failed to load EthernetScanner library! " << dlerror() << '\n';
        return false;
    }

    /* --- resolve every symbol, abort if any is missing --- */
    if (!LOAD_SYMBOL(EthernetScanner_Connect)            ||
        !LOAD_SYMBOL(EthernetScanner_Disconnect)         ||
        !LOAD_SYMBOL(EthernetScanner_GetConnectStatus)   ||
        !LOAD_SYMBOL(EthernetScanner_GetXZIExtended)     ||
        !LOAD_SYMBOL(EthernetScanner_GetImage)           ||
        !LOAD_SYMBOL(EthernetScanner_GetDllFiFoState)    ||
        !LOAD_SYMBOL(EthernetScanner_ResetDllFiFo)       ||
        !LOAD_SYMBOL(EthernetScanner_GetVersion)         ||
        !LOAD_SYMBOL(EthernetScanner_WriteData)          ||
        !LOAD_SYMBOL(EthernetScanner_ReadData))
    {
        std::cerr << " One or more DLL symbols could not be resolved.\n";
        unloadEthernetScannerLibrary();
        return false;
    }

    p_Connect           =p_EthernetScanner_Connect;
    p_Disconnect        =p_EthernetScanner_Disconnect;
    p_GetConnectStatus  =p_EthernetScanner_GetConnectStatus;
    p_GetXZIExtended    =p_EthernetScanner_GetXZIExtended;
    p_GetImage          =p_EthernetScanner_GetImage;
    p_GetDllFiFoState   =p_EthernetScanner_GetDllFiFoState;
    p_ResetDllFiFo      =p_EthernetScanner_ResetDllFiFo;
    p_GetVersion        =p_EthernetScanner_GetVersion;
    p_WriteData         =p_EthernetScanner_WriteData;
    p_ReadData          =p_EthernetScanner_ReadData;


    std::cout << "All EthernetScanner symbols loaded successfully.\n";
    return true;
}



/* ───────── Exception class ───────── */
class SensorException : public std::runtime_error {
public: explicit SensorException(const std::string& m) : std::runtime_error(m) {}
};

/* ───────── Sensor wrapper (same interface) ───────── */
class Sensor {
public:
    Sensor(const std::string& ip, int port);
    ~Sensor();

    void connect(int timeout_ms = 0);
    void disconnect();

    int  get_connect_status() const;
    ScannedProfile get_scanned_profile(int timeout_ms = 1000);

    void write_data(const std::string& cmd);
    std::string read_data(const std::string& cmd, int cache = 0);

private:
    void allocate();
    void release();

    std::string ip_, port_;
    void* handle_{nullptr};

    /* raw buffers */
    double *roiX_{}, *roiZ_{};
    int    *inten_{}, *sigW_{};
    uint32_t *encVal_{}, *uio_{};
    int *picCnt_{};

    static constexpr size_t READ_SZ = 128*1024;
    char readBuf_[READ_SZ]{};
};

/* ---------- implementation (same logic as before) ----------------- */
Sensor::Sensor(const std::string& ip, int port)
    : ip_(ip), port_(std::to_string(port)) {}

Sensor::~Sensor() { if (handle_) disconnect(); }

void Sensor::connect(int timeout_ms)
{
    std::vector<char> ipBuf (ip_.begin(),  ip_.end());  ipBuf .push_back('\0');
    std::vector<char> portBuf(port_.begin(),port_.end());portBuf.push_back('\0');

    handle_ = p_Connect(ipBuf.data(), portBuf.data(), timeout_ms);
    auto t0 = std::chrono::steady_clock::now();
    while (!handle_ &&
           (timeout_ms == 0 ||                          // 0 → keep trying forever
            std::chrono::steady_clock::now() - t0 <
            std::chrono::milliseconds(timeout_ms)))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        handle_ = p_Connect(ipBuf.data(), portBuf.data(), 0);
    }
    if (!handle_)
        throw SensorException("Connect failed: no handle from DLL");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // scanner init
    if (get_connect_status() != SENSOR_CONNECTED)
        throw SensorException("Connect failed: status != CONNECTED");
    allocate();
}
void Sensor::disconnect()
{
    handle_ = p_Disconnect(handle_);
    release();
}
int Sensor::get_connect_status() const
{
    int st{}; p_GetConnectStatus(handle_, &st);
    return (st & SENSOR_CONNECTED) ? SENSOR_CONNECTED : SENSOR_DISCONNECTED;
}
void Sensor::allocate()
{
    roiX_ = new double[SENSOR_BUFFERSIZEMAX]{};
    roiZ_ = new double[SENSOR_BUFFERSIZEMAX]{};
    inten_= new int   [SENSOR_BUFFERSIZEMAX]{};
    sigW_ = new int   [SENSOR_BUFFERSIZEMAX]{};
    encVal_ = new uint32_t(0);
    uio_    = new uint32_t(0);
    picCnt_ = new int(0);
}
void Sensor::release()
{
    delete[] roiX_;  delete[] roiZ_;  delete[] inten_; delete[] sigW_;
    delete encVal_;  delete uio_;     delete picCnt_;
}
ScannedProfile Sensor::get_scanned_profile(int timeout_ms)
{
    int resp = p_GetXZIExtended(
        handle_,
        roiX_, roiZ_, inten_, sigW_, SENSOR_BUFFERSIZEMAX,
        encVal_, reinterpret_cast<unsigned char*>(uio_),
        timeout_ms, nullptr, 0, picCnt_);

    if (resp < 0) throw SensorException("GetXZIExtended error " + std::to_string(resp));

    ScannedProfile out;
    out.scannedPoints = resp;
    out.encoderValue  = *encVal_;
    out.pictureCounter= *picCnt_;

    out.roiWidthX .assign(roiX_, roiX_ + resp);
    out.roiHeightZ.assign(roiZ_, roiZ_ + resp);

    uint32_t u = *uio_;
    out.userIOState = { int(u&1), int((u>>1)&1), int((u>>2)&1), int((u>>3)&1),
                        int((u>>4)&1), int((u>>5)&1), int((u>>6)&1) };

    return out;
}
void Sensor::write_data(const std::string& cmd)
{
    std::vector<char> buf(cmd.begin(), cmd.end()); buf.push_back('\0');
    int rc = p_WriteData(handle_, buf.data(), cmd.size());
    if (rc != static_cast<int>(cmd.size()))
        throw SensorException("WriteData failed: " + cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}
std::string Sensor::read_data(const std::string& cmd, int cache)
{
    std::vector<char> c(cmd.begin(), cmd.end()); c.push_back('\0');
    int rc = p_ReadData(handle_, c.data(), readBuf_, READ_SZ, cache);
    if (rc != SENSOR_READDATAOK)
        throw SensorException("ReadData failed: " + cmd);
    return std::string(readBuf_);
}

/* ────────────────────────────────────────────────────────────────── */
/*                               main                                 */
/* ─────────────────────────────────────────────────────────────────── */
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signal_handler);

    auto node = rclcpp::Node::make_shared("profile_publisher_node");
    auto pub  = node->create_publisher<sensor_msgs::msg::PointCloud2>(
                    "/wecat3d/pointcloud", 10);

    /* ---- open PCD file with placeholder header (unchanged) ---- */
    std::ofstream pcd("merged_encoder_output.pcd", std::ios::trunc);
    pcd << "# .PCD v0.7 - Point Cloud Data file\nVERSION 0.7\nFIELDS x y z\n"
        << "SIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        << "WIDTH 0000000000\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        << "POINTS 0000000000\nDATA ascii\n";
    std::streampos header_start = 0;

    uint64_t total_points = 0;

    if (!loadEthernetScannerLibrary()) return 1;

    try {
        Sensor sensor("192.168.100.1", 32001);
        sensor.connect(3000);

        sensor.write_data("SetHeartbeat=1000");
        sensor.write_data("SetExposureTime=750");
        sensor.write_data("SetTriggerSource=2");
        sensor.write_data("SetTriggerEncoderStep=20");
        sensor.write_data("SetEncoderTriggerFunction=2");
        sensor.write_data("ResetEncoder");
        sensor.write_data("SetRangeImageNrProfiles=1");
        sensor.write_data("SetAcquisitionStart");

        /* ROS message scaffold */
        sensor_msgs::msg::PointCloud2 msg;
        sensor_msgs::PointCloud2Modifier mod(msg);
        mod.setPointCloud2FieldsByString(1,"xyz");
        msg.header.frame_id = "map";
        msg.height = 1; msg.is_dense = false; msg.is_bigendian = false;
        const float to_m = 0.001f;

        int64_t last_enc = std::numeric_limits<int64_t>::min();
        int scanNo = 0;

        while (!stop_flag) {
            try {
                ScannedProfile scan = sensor.get_scanned_profile(1000);
                ++scanNo;

                const auto& X = scan.roiWidthX;
                const auto& Z = scan.roiHeightZ;
                size_t n_valid = scan.scannedPoints;          // ← FIX #1

                /* signed encoder */
                int64_t enc = scan.encoderValue < (1u<<31)
                            ? scan.encoderValue
                            : (int64_t)scan.encoderValue - (1ll<<32);

                if (last_enc == std::numeric_limits<int64_t>::min() || enc != last_enc)
                {
                    double y_mm = enc * ENC_SCALE_MM;

                    /* write points */
                    for (size_t i=0; i<n_valid; ++i)
                        pcd << std::fixed << std::setprecision(6)
                            << X[i] << ' ' << y_mm << ' ' << Z[i] << '\n';
                    total_points += n_valid;                   // ← keeps header right

                    /* ROS message */
                    msg.header.stamp = node->now();
                    if (msg.width != n_valid) { msg.width = n_valid; mod.resize(n_valid); }  // ← FIX #3

                    sensor_msgs::PointCloud2Iterator<float>
                        it_x(msg,"x"), it_y(msg,"y"), it_z(msg,"z");
                    float y_m = static_cast<float>(y_mm)*to_m;
                    for (size_t i=0;i<n_valid;++i,++it_x,++it_y,++it_z)
                    {
                        *it_x = X[i]*to_m;
                        *it_y = y_m;
                        *it_z = Z[i]*to_m;
                    }
                    pub->publish(msg);

                    if (scanNo % 500 == 0)
                        std::cout << "Scans: " << scanNo
                                << "  Points: " << total_points << '\n';
                }
                last_enc = enc;

                rclcpp::spin_some(node);

            } catch (const SensorException& e) {
                if (std::string(e.what()).find("-1") != std::string::npos) {
                    continue;
                } else {
                    throw;
                }
            }
        }
        /* ---- rewrite header with real counts ---- */
        pcd.seekp(header_start);
        pcd << "# .PCD v0.7 - Point Cloud Data file\nVERSION 0.7\nFIELDS x y z\n"
            << "SIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
            << "WIDTH "  << std::setw(10) << std::setfill('0') << total_points << '\n'
            << "HEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS "
            << std::setw(10) << std::setfill('0') << total_points << '\n'
            << "DATA ascii\n";
        pcd.close();

        sensor.write_data("SetAcquisitionStop");
        sensor.write_data("SetResetSettings");
        sensor.disconnect();
    }
    catch (const std::exception& e) {
        std::cerr << "Fatal: " << e.what() << '\n';
    }

    unloadEthernetScannerLibrary();
    rclcpp::shutdown();
    return 0;
}