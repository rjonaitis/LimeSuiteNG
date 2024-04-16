#pragma once

#include "comms/USB/USBGeneric.h"
#include "limesuiteng/config.h"
#include "USBTransferContext_FT601.h"

#ifndef __unix__
    #include "FTD3XXLibrary/FTD3XX.h"
#endif

namespace lime {

/** @brief A class for communicating with devices using the FTDI FT601 USB controller. */
class FT601 : public USBGeneric
{
  public:
    /**
      @brief Constructs the class for communicating with the FT601 controller.
      @param usbContext The USB context to use for the communication.
     */
    FT601(void* usbContext = nullptr);
    ~FT601();

    bool Connect(uint16_t vid, uint16_t pid, const std::string& serial = "") override;
    void Disconnect() override;

#ifndef __unix__
    int32_t BulkTransfer(uint8_t endPoint, uint8_t* data, int length, int32_t timeout = USBGeneric::defaultTimeout) override;
#endif

    int32_t ControlTransfer(int requestType,
        int request,
        int value,
        int index,
        uint8_t* data,
        uint32_t length,
        int32_t timeout = USBGeneric::defaultTimeout) override;

#ifndef __unix__
    int BeginDataXfer(uint8_t* buffer, uint32_t length, uint8_t endPointAddr) override;
    bool WaitForXfer(int contextHandle, int32_t timeout_ms) override;
    int FinishDataXfer(uint8_t* buffer, uint32_t length, int contextHandle) override;
    void AbortEndpointXfers(uint8_t endPointAddr) override;
#endif

    /**
      @brief Resets the stream buffers of the device.
      @return Status of the operation (0 - success; -1 - failure).
     */
    int ResetStreamBuffers();

  protected:
#ifndef __unix__
    FT_HANDLE mFTHandle;
    int ReinitPipe(unsigned char ep);
    void WaitForXfers(uint8_t endPointAddr) override;
#else
    int FT_SetStreamPipe(unsigned char ep, size_t size);
    int FT_FlushPipe(unsigned char ep);
    uint32_t mUsbCounter;
#endif

    int GetUSBContextIndex() override;
};

} // namespace lime
