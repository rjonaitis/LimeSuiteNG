#ifndef LIME_LIMESDR_MINI_H
#define LIME_LIMESDR_MINI_H

#include "LMS7002M_SDRDevice.h"

#include <vector>
#include <memory>

namespace lime {

class IComms;
class ISerialPort;
class IUSB;

/** @brief Class for managing the LimeSDR Mini device. */
class LimeSDR_Mini : public LMS7002M_SDRDevice
{
  public:
    LimeSDR_Mini(std::shared_ptr<IComms> spiLMS,
        std::shared_ptr<IComms> spiFPGA,
        std::shared_ptr<IUSB> mStreamPort,
        std::shared_ptr<ISerialPort> commsPort);
    ~LimeSDR_Mini();

    OpStatus Configure(const SDRConfig& config, uint8_t moduleIndex) override;

    OpStatus Init() override;
    OpStatus Reset() override;

    OpStatus SetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double sampleRate, uint8_t oversample) override;

    double GetClockFreq(uint8_t clk_id, uint8_t channel) override;
    OpStatus SetClockFreq(uint8_t clk_id, double freq, uint8_t channel) override;

    double GetTemperature(uint8_t moduleIndex) override;

    OpStatus Synchronize(bool toChip) override;

    OpStatus SPI(uint32_t chipSelect, const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override;

    OpStatus GPIODirRead(uint8_t* buffer, const size_t bufLength) override;
    OpStatus GPIORead(uint8_t* buffer, const size_t bufLength) override;
    OpStatus GPIODirWrite(const uint8_t* buffer, const size_t bufLength) override;
    OpStatus GPIOWrite(const uint8_t* buffer, const size_t bufLength) override;

    OpStatus CustomParameterWrite(const std::vector<CustomParameterIO>& parameters) override;
    OpStatus CustomParameterRead(std::vector<CustomParameterIO>& parameters) override;

    OpStatus UploadMemory(
        eMemoryDevice device, uint8_t moduleIndex, const char* data, size_t length, UploadMemoryCallback callback) override;

    void SetSerialNumber(const std::string& number);
    OpStatus SetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t path) override;

    std::unique_ptr<lime::RFStream> StreamCreate(const StreamConfig& config, uint8_t moduleIndex) override;

  private:
    static OpStatus UpdateFPGAInterface(void* userData);
    OpStatus SetRFSwitch(TRXDir dir, uint8_t path);

    std::shared_ptr<IUSB> mStreamPort;
    std::shared_ptr<ISerialPort> mSerialPort;
    std::shared_ptr<IComms> mlms7002mPort;
    std::shared_ptr<IComms> mfpgaPort;
    bool mConfigInProgress{};
};

} // namespace lime

#endif // LIME_LIMESDR_MINI_H
