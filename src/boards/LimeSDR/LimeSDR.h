#ifndef LIME_LIMESDR_H
#define LIME_LIMESDR_H

#include "LMS7002M_SDRDevice.h"
#include "limesuite/DeviceRegistry.h"
#include "limesuite/DeviceHandle.h"
#include "protocols/LMS64CProtocol.h"
#include "USBCommon.h"
#include <vector>
#include <memory>

#include "dataTypes.h"

namespace lime {

class USBGeneric;
class LMS7002M;
class Streamer;
class FPGA;
class TRXLooper_USB;

class LimeSDR : public LMS7002M_SDRDevice
{
  public:
    LimeSDR(std::shared_ptr<IComms> spiLMS,
        std::shared_ptr<IComms> spiFPGA,
        std::shared_ptr<USBGeneric> mStreamPort,
        std::shared_ptr<ISerialPort> commsPort);
    virtual ~LimeSDR();

    virtual void Configure(const SDRConfig& config, uint8_t moduleIndex) override;

    virtual int Init() override;
    virtual void Reset() override;

    virtual double GetClockFreq(uint8_t clk_id, uint8_t channel) override;
    virtual void SetClockFreq(uint8_t clk_id, double freq, uint8_t channel) override;

    virtual void Synchronize(bool toChip) override;
    virtual void EnableCache(bool enable) override;

    virtual void SPI(uint32_t chipSelect, const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override;

    virtual int StreamSetup(const StreamConfig& config, uint8_t moduleIndex) override;

    virtual void StreamStart(uint8_t moduleIndex) override;
    virtual void StreamStop(uint8_t moduleIndex) override;

    virtual void StreamStatus(uint8_t moduleIndex, SDRDevice::StreamStats* rx, SDRDevice::StreamStats* tx) override;

    virtual void* GetInternalChip(uint32_t index) override;

    virtual int GPIODirRead(uint8_t* buffer, const size_t bufLength) override;
    virtual int GPIORead(uint8_t* buffer, const size_t bufLength) override;
    virtual int GPIODirWrite(const uint8_t* buffer, const size_t bufLength) override;
    virtual int GPIOWrite(const uint8_t* buffer, const size_t bufLength) override;

    virtual int CustomParameterWrite(
        const int32_t* ids, const double* values, const size_t count, const std::string& units) override;
    virtual int CustomParameterRead(
        const int32_t* ids, double* values, const size_t count, std::vector<std::string>& units) override;

    virtual int ReadFPGARegister(uint32_t address);
    virtual int WriteFPGARegister(uint32_t address, uint32_t value);

  protected:
    int EnableChannel(TRXDir dir, uint8_t channel, bool enabled);
    SDRDevice::Descriptor GetDeviceInfo();
    void ResetUSBFIFO();
    void SetSampleRate(double f_Hz, uint8_t oversample);
    static int UpdateFPGAInterface(void* userData);

  private:
    std::shared_ptr<USBGeneric> mStreamPort;
    std::shared_ptr<ISerialPort> mSerialPort;
    std::shared_ptr<IComms> mlms7002mPort;
    std::shared_ptr<IComms> mfpgaPort;
};

class LimeSDREntry : public USBEntry
{
  public:
    LimeSDREntry();

#ifndef __unix__
    virtual std::vector<DeviceHandle> enumerate(const DeviceHandle& hint) override;
#endif
    virtual SDRDevice* make(const DeviceHandle& handle) override;
};

} // namespace lime

#endif /* LIME_LIMESDR_H */
