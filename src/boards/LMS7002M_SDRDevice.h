#ifndef LIME_LMS7002M_SDRDevice_H
#define LIME_LMS7002M_SDRDevice_H

#include "PacketsFIFO.h"
#include "dataTypes.h"

#include <vector>
#include <unordered_map>
#include <functional>
#include <string.h>

#include "limesuite/SDRDevice.h"

namespace lime {

class LMS7002M;
class TRXLooper;
class FPGA;

// Base class for device with multiple LMS7002M chips and FPGA
class LIME_API LMS7002M_SDRDevice : public SDRDevice
{
public:
    LMS7002M_SDRDevice();
    virtual ~LMS7002M_SDRDevice();

    virtual void Configure(const SDRConfig config, uint8_t moduleIndex) = 0;

    /// Returns SPI slave names and chip select IDs for use with SDRDevice::SPI()
    virtual const Descriptor &GetDescriptor() const = 0;

    virtual int Init() = 0;
    virtual void Reset() override;

    //virtual double GetRate(Dir dir, uint8_t channel) const = 0;

    virtual double GetClockFreq(uint8_t clk_id, uint8_t channel) = 0;
    virtual void SetClockFreq(uint8_t clk_id, double freq, uint8_t channel) = 0;

    virtual void Synchronize(bool toChip) override;
    virtual void EnableCache(bool enable) override;

    virtual int StreamSetup(const StreamConfig &config, uint8_t moduleIndex) = 0;
    virtual void StreamStart(uint8_t moduleIndex) override;
    virtual void StreamStop(uint8_t moduleIndex) override;

    virtual int StreamRx(uint8_t channel, void **samples, uint32_t count, StreamMeta *meta) override;
    virtual int StreamTx(uint8_t channel, const void **samples, uint32_t count,
                         const StreamMeta *meta) override;
    virtual void StreamStatus(uint8_t channel, SDRDevice::StreamStats &status) = 0;

    virtual void SPI(uint32_t spiBusAddress, const uint32_t *MOSI, uint32_t *MISO,
                     uint32_t count) override;

    virtual int I2CWrite(int address, const uint8_t *data, uint32_t length) override;
    virtual int I2CRead(int addr, uint8_t *dest, uint32_t length) override;
    virtual int GPIOWrite(const uint8_t *buffer, const size_t bufLength) override;
    virtual int GPIORead(uint8_t *buffer, const size_t bufLength) override;
    virtual int GPIODirWrite(const uint8_t *buffer, const size_t bufLength) override;
    virtual int GPIODirRead(uint8_t *buffer, const size_t bufLength) override;
    virtual int CustomParameterWrite(const int32_t *ids, const double *values, const size_t count, const std::string& units) override;
    virtual int CustomParameterRead(const int32_t *ids, double *values, const size_t count, std::string* units) override;

    virtual void SetDataLogCallback(DataCallbackType callback) override;
    virtual void SetMessageLogCallback(LogCallbackType callback) override;

    virtual void *GetInternalChip(uint32_t index);
    virtual void SetFPGAInterfaceFreq(uint8_t interp, uint8_t dec, double txPhase, double rxPhase) = 0;

    virtual bool UploadMemory(uint32_t id, const char* data, size_t length, UploadMemoryCallback callback) override;

protected:
    DataCallbackType mCallback_logData;
    LogCallbackType mCallback_logMessage;
    std::vector<LMS7002M*> mLMSChips;
    std::vector<TRXLooper*> mStreamers;

    StreamConfig mStreamConfig;
    FPGA *mFPGA;

private:
    friend class DeviceRegistry;
};

}
#endif
