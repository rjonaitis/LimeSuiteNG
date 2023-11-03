#include "LimeSDR_Mini.h"
#include "DeviceExceptions.h"
#include "limesuite/DeviceRegistry.h"
#include "limesuite/DeviceHandle.h"
#include "protocols/LMS64CProtocol.h"
#include "Logger.h"
#include "USBCommon.h"

#include "FT601/FT601.h"

#ifndef __unix__
    #include "windows.h"
    #include "FTD3XXLibrary/FTD3XX.h"
#else
    #ifdef __GNUC__
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wpedantic"
    #endif
    #include <libusb.h>
    #ifdef __GNUC__
        #pragma GCC diagnostic pop
    #endif
    #include <mutex>
#endif

using namespace lime;

void __loadLimeSDR_Mini(void) // TODO: fixme replace with LoadLibrary/dlopen
{
    static LimeSDR_MiniEntry limesdr_miniSupport; // Self register on initialization
}

// Device identifier vendor ID and product ID pairs.
const std::set<VidPid> ids{ { 1027, 24607 } };

LimeSDR_MiniEntry::LimeSDR_MiniEntry()
    : USBEntry("LimeSDR_Mini", ids)
{
}

#ifndef __unix__
std::vector<DeviceHandle> LimeSDR_MiniEntry::enumerate(const DeviceHandle& hint)
{
    std::vector<DeviceHandle> handles;

    if (!hint.media.empty() && hint.media.find("USB") == std::string::npos)
    {
        return handles;
    }

    FT_STATUS ftStatus = FT_OK;
    static DWORD numDevs = 0;

    ftStatus = FT_CreateDeviceInfoList(&numDevs);

    if (!FT_FAILED(ftStatus) && numDevs > 0)
    {
        DWORD Flags = 0;
        char SerialNumber[16] = { 0 };
        char Description[32] = { 0 };
        for (DWORD i = 0; i < numDevs; i++)
        {
            ftStatus = FT_GetDeviceInfoDetail(i, &Flags, nullptr, nullptr, nullptr, SerialNumber, Description, nullptr);
            if (!FT_FAILED(ftStatus))
            {
                ConnectionHandle handle;
                handle.media = Flags & FT_FLAGS_SUPERSPEED ? "USB 3" : Flags & FT_FLAGS_HISPEED ? "USB 2" : "USB";
                handle.name = Description;
                handle.index = i;
                handle.serial = SerialNumber;
                // Add handle conditionally, filter by serial number
                if (hint.serial.empty() || handle.serial.find(hint.serial) != std::string::npos)
                    handles.push_back(handle);
            }
        }
    }

    return handles;
}
#endif

static constexpr int streamBulkWriteAddr = 0x03;
static constexpr int streamBulkReadAddr = 0x83;

static constexpr int ctrlBulkWriteAddr = 0x02;
static constexpr int ctrlBulkReadAddr = 0x82;

class USB_CSR_Pipe_Mini : public USB_CSR_Pipe
{
  public:
    explicit USB_CSR_Pipe_Mini(FT601& port)
        : USB_CSR_Pipe()
        , port(port){};

    virtual int Write(const uint8_t* data, size_t length, int timeout_ms) override
    {
        return port.BulkTransfer(ctrlBulkWriteAddr, const_cast<uint8_t*>(data), length, timeout_ms);
    }

    virtual int Read(uint8_t* data, size_t length, int timeout_ms) override
    {
        return port.BulkTransfer(ctrlBulkReadAddr, data, length, timeout_ms);
    }

  protected:
    FT601& port;
};

SDRDevice* LimeSDR_MiniEntry::make(const DeviceHandle& handle)
{
    const auto splitPos = handle.addr.find(":");
    const uint16_t vid = std::stoi(handle.addr.substr(0, splitPos), nullptr, 16);
    const uint16_t pid = std::stoi(handle.addr.substr(splitPos + 1), nullptr, 16);

    FT601* usbComms = new FT601(ctx);
    if (usbComms->Connect(vid, pid, handle.serial) != 0)
    {
        delete usbComms;
        char reason[256];
        sprintf(reason, "Unable to connect to device using handle(%s)", handle.Serialize().c_str());
        throw std::runtime_error(reason);
    }

    USB_CSR_Pipe* usbPipe = new USB_CSR_Pipe_Mini(*usbComms);

    // protocol layer
    IComms* route_lms7002m = new LMS64C_LMS7002M_Over_USB(*usbPipe);
    IComms* route_fpga = new LMS64C_FPGA_Over_USB(*usbPipe);

    return new LimeSDR_Mini(route_lms7002m, route_fpga, usbComms, usbPipe);
}
