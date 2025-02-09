#include "LimeSDR.h"

#include "limesuiteng/LMS7002M.h"
#include "limesuiteng/Logger.h"

#include "boards/LimeSDR/LMS64C_ADF_Over_USB.h"
#include "comms/IComms.h"
#include "comms/ISerialPort.h"
#include "comms/USB/FX3/FX3.h"
#include "comms/USB/IUSB.h"
#include "comms/USB/USBDMAEmulation.h"
#include "chips/LMS7002M/LMS7002MCSR_Data.h"
#include "chips/LMS7002M/validation.h"
#include "chips/Si5351C/Si5351C.h"
#include "FPGA/FPGA_common.h"
#include "protocols/LMS64CProtocol.h"
#include "protocols/LMSBoards.h"
#include "protocols/LMS64CProtocol.h"
#include "streaming/TRXLooper.h"
#include "utilities/toString.h"

#include "DeviceTreeNode.h"

#include <array>
#include <cassert>
#include <cmath>
#include <memory>

using namespace lime::LMS64CProtocol;
using namespace lime::LMS7002MCSR_Data;
using namespace std::literals::string_literals;

namespace lime {
namespace limesdrusb {

static const uint8_t SPI_LMS7002M = 0;
static const uint8_t SPI_FPGA = 1;
static const uint8_t SPI_ADF4002 = 2;

static const CustomParameter CP_VCTCXO_DAC = { "VCTCXO DAC (volatile)"s, 0, 0, 65535, false };
static const CustomParameter CP_TEMPERATURE = { "Board Temperature"s, 1, 0, 65535, true };

static const std::vector<std::pair<uint16_t, uint16_t>> lms7002defaultsOverrides = { //
    { 0x0022, 0x0FFF },
    { 0x0023, 0x5550 },
    { 0x002B, 0x0038 },
    { 0x002C, 0x0000 },
    { 0x002D, 0x0641 },
    { 0x0086, 0x4101 },
    { 0x0087, 0x5555 },
    { 0x0088, 0x0525 },
    { 0x0089, 0x1078 },
    { 0x008B, 0x218C },
    { 0x008C, 0x267B },
    { 0x00A6, 0x000F },
    { 0x00A9, 0x8000 },
    { 0x00AC, 0x2000 },
    { 0x0108, 0x218C },
    { 0x0109, 0x57C1 },
    { 0x010A, 0x154C },
    { 0x010B, 0x0001 },
    { 0x010C, 0x8865 },
    { 0x010D, 0x011A },
    { 0x010E, 0x0000 },
    { 0x010F, 0x3142 },
    { 0x0110, 0x2B14 },
    { 0x0111, 0x0000 },
    { 0x0112, 0x000C },
    { 0x0113, 0x03C2 },
    { 0x0114, 0x01F0 },
    { 0x0115, 0x000D },
    { 0x0118, 0x418C },
    { 0x0119, 0x528C },
    { 0x011A, 0x3001 },
    { 0x011C, 0x8941 },
    { 0x011D, 0x0000 },
    { 0x011E, 0x0984 },
    { 0x0120, 0xE6C0 },
    { 0x0121, 0x3638 },
    { 0x0122, 0x0514 },
    { 0x0123, 0x200F },
    { 0x0200, 0x00E1 },
    { 0x0208, 0x017B },
    { 0x020B, 0x4000 },
    { 0x020C, 0x8000 },
    { 0x0400, 0x8081 },
    { 0x0404, 0x0006 },
    { 0x040B, 0x1020 },
    { 0x040C, 0x00FB }
};

} // namespace limesdrusb

/// @brief Constructs a new LimeSDR object
/// @param spiLMS The communications port to the LMS7002M chip.
/// @param spiFPGA The communications port to the device's FPGA.
/// @param streamPort The communications port to send and receive sample data.
/// @param commsPort The communications port for direct communications with the device.
LimeSDR::LimeSDR(std::shared_ptr<IComms> spiLMS,
    std::shared_ptr<IComms> spiFPGA,
    std::shared_ptr<IUSB> streamPort,
    std::shared_ptr<ISerialPort> commsPort)
    : mStreamPort(streamPort)
    , mSerialPort(commsPort)
    , mlms7002mPort(spiLMS)
    , mfpgaPort(spiFPGA)
    , mADF(std::make_unique<ADF4002>())
    , mConfigInProgress(false)
{
    mStreamers.resize(1);
    SDRDescriptor descriptor = GetDeviceInfo();

    mFPGA = std::make_unique<FPGA>(spiFPGA, spiLMS);
    FPGA::GatewareInfo gw = mFPGA->GetGatewareInfo();
    FPGA::GatewareToDescriptor(gw, descriptor);

    {
        RFSOCDescriptor soc = GetDefaultLMS7002MDescriptor();
        descriptor.rfSOC.push_back(soc);

        std::unique_ptr<LMS7002M> chip = std::make_unique<LMS7002M>(mlms7002mPort);
        chip->ModifyRegistersDefaults(limesdrusb::lms7002defaultsOverrides);
        chip->SetConnection(mlms7002mPort);
        chip->SetOnCGENChangeCallback(UpdateFPGAInterface, this);
        mLMSChips.push_back(std::move(chip));
    }

    auto fpgaNode = std::make_shared<DeviceTreeNode>("FPGA"s, eDeviceTreeNodeClass::FPGA, mFPGA.get());
    fpgaNode->children.push_back(
        std::make_shared<DeviceTreeNode>("LMS7002"s, eDeviceTreeNodeClass::LMS7002M, mLMSChips.at(0).get()));
    descriptor.socTree = std::make_shared<DeviceTreeNode>("LimeSDR-USB"s, eDeviceTreeNodeClass::SDRDevice, this);
    descriptor.socTree->children.push_back(fpgaNode);

    auto ADFComms = std::make_shared<LMS64C_ADF_Over_USB>(mSerialPort, 0);
    mADF->Initialize(ADFComms, 30.72e6);
    descriptor.socTree->children.push_back(std::make_shared<DeviceTreeNode>("ADF4002", eDeviceTreeNodeClass::ADF4002, mADF.get()));

    descriptor.memoryDevices[ToString(eMemoryDevice::FPGA_FLASH)] = std::make_shared<DataStorage>(this, eMemoryDevice::FPGA_FLASH);
    const std::unordered_map<std::string, Region> eepromMap = { { "VCTCXO_DAC"s, { 0x0010, 1 } } };
    descriptor.memoryDevices[ToString(eMemoryDevice::EEPROM)] =
        std::make_shared<DataStorage>(this, eMemoryDevice::EEPROM, eepromMap);

    descriptor.customParameters.push_back(limesdrusb::CP_VCTCXO_DAC);
    descriptor.customParameters.push_back(limesdrusb::CP_TEMPERATURE);
    descriptor.spiSlaveIds = { { "LMS7002M"s, limesdrusb::SPI_LMS7002M }, { "FPGA"s, limesdrusb::SPI_FPGA } };

    mDeviceDescriptor = descriptor;

    //must configure synthesizer before using LimeSDR
    /*if (info.device == LMS_DEV_LIMESDR && info.hardware < 4)
    {
        auto si5351module = std::make_shared<Si5351C>();
        si5351module->Initialize(conn);
        si5351module->SetPLL(0, 25000000, 0);
        si5351module->SetPLL(1, 25000000, 0);
        si5351module->SetClock(0, 27000000, true, false);
        si5351module->SetClock(1, 27000000, true, false);
        for (int i = 2; i < 8; ++i)
            si5351module->SetClock(i, 27000000, false, false);
        Si5351C::Status status = si5351module->ConfigureClocks();
        if (status != Si5351C::SUCCESS)
        {
            lime::warning("Failed to configure Si5351C"s);
            return;
        }
        status = si5351module->UploadConfiguration();
        if (status != Si5351C::SUCCESS)
            lime::warning("Failed to upload Si5351C configuration"s);
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); //some settle time
    }*/
}

LimeSDR::~LimeSDR()
{
}

OpStatus LimeSDR::Configure(const SDRConfig& cfg, uint8_t moduleIndex = 0)
{
    auto& chip = mLMSChips.at(0);

    mConfigInProgress = true;
    if (!cfg.skipDefaults)
    {
        Init();
    }
    // enabled ADC/DAC is required for FPGA to work
    chip->Modify_SPI_Reg_bits(PD_RX_AFE1, 0);
    chip->Modify_SPI_Reg_bits(PD_TX_AFE1, 0);

    OpStatus status = LMS7002M_Configure(*chip, cfg);
    mConfigInProgress = false;

    if (status != OpStatus::Success)
        return status;

    double sampleRate{ 0 };
    bool rxUsed = false;
    bool txUsed = false;
    for (int i = 0; i < 2; ++i)
    {
        const ChannelConfig& ch = cfg.channel[i];
        rxUsed |= ch.rx.enabled;
        txUsed |= ch.tx.enabled;
    }
    if (rxUsed)
        sampleRate = cfg.channel[0].rx.sampleRate;
    else if (txUsed)
        sampleRate = cfg.channel[0].tx.sampleRate;

    if (sampleRate > 0)
    {
        status = LMS7002M_SDRDevice::LMS7002M_SetSampleRate(sampleRate, cfg.channel[0].rx.oversample, cfg.channel[0].tx.oversample);
        if (status != OpStatus::Success)
            return status;
    }

    for (int c = 0; c < 2; ++c)
    {
        LMS7002M_SDRDevice::SetAntenna(0, TRXDir::Tx, c, cfg.channel[c].tx.path);
        LMS7002M_SDRDevice::SetAntenna(0, TRXDir::Rx, c, cfg.channel[c].rx.path);
        LMS7002ChannelCalibration(*chip, cfg.channel[c], c);
    }

    if (sampleRate > 0)
    {
        status = UpdateFPGAInterface(this);
        if (status != OpStatus::Success)
            return lime::ReportError(OpStatus::Error, "LimeSDR: failed to update FPGA interface frequency."s);
    }
    return OpStatus::Success;
}

// Callback for updating FPGA's interface clocks when LMS7002M CGEN is manually modified
OpStatus LimeSDR::UpdateFPGAInterface(void* userData)
{
    assert(userData != nullptr);
    LimeSDR* pthis = static_cast<LimeSDR*>(userData);
    // don't care about cgen changes while doing Config(), to avoid unnecessary fpga updates
    if (pthis->mConfigInProgress)
        return OpStatus::Success;
    return UpdateFPGAInterfaceFrequency(*pthis->mLMSChips.at(0), *pthis->mFPGA, 0);
}

OpStatus LimeSDR::SetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double sampleRate, uint8_t oversample)
{
    return LMS7002M_SDRDevice::LMS7002M_SetSampleRate(sampleRate, oversample, oversample);
}

OpStatus LimeSDR::Init()
{
    OpStatus status;
    auto& lms = mLMSChips.at(0);
    // TODO: write GPIO to hard reset the chip
    status = lms->ResetChip();
    if (status != OpStatus::Success)
        return status;

    return status;
}

SDRDescriptor LimeSDR::GetDeviceInfo(void)
{
    assert(mSerialPort);
    SDRDescriptor deviceDescriptor;

    LMS64CProtocol::FirmwareInfo info{};
    OpStatus returnCode = LMS64CProtocol::GetFirmwareInfo(*mSerialPort, info);

    if (returnCode != OpStatus::Success)
    {
        deviceDescriptor.name = GetDeviceName(LMS_DEV_UNKNOWN);
        deviceDescriptor.expansionName = GetExpansionBoardName(EXP_BOARD_UNKNOWN);

        return deviceDescriptor;
    }

    deviceDescriptor.name = GetDeviceName(static_cast<eLMS_DEV>(info.deviceId));
    deviceDescriptor.expansionName = GetExpansionBoardName(static_cast<eEXP_BOARD>(info.expansionBoardId));
    deviceDescriptor.firmwareVersion = std::to_string(info.firmware);
    deviceDescriptor.hardwareVersion = std::to_string(info.hardware);
    deviceDescriptor.protocolVersion = std::to_string(info.protocol);
    deviceDescriptor.serialNumber = info.boardSerialNumber;

    const uint32_t addrs[] = { 0x0000, 0x0001, 0x0002, 0x0003 };
    uint32_t data[4];
    SPI(limesdrusb::SPI_FPGA, addrs, data, 4);
    auto boardID = static_cast<eLMS_DEV>(data[0]); //(pkt.inBuffer[2] << 8) | pkt.inBuffer[3];
    auto gatewareVersion = data[1]; //(pkt.inBuffer[6] << 8) | pkt.inBuffer[7];
    auto gatewareRevision = data[2]; //(pkt.inBuffer[10] << 8) | pkt.inBuffer[11];
    auto hwVersion = data[3] & 0x7F; //pkt.inBuffer[15]&0x7F;

    deviceDescriptor.gatewareTargetBoard = GetDeviceName(boardID);
    deviceDescriptor.gatewareVersion = std::to_string(gatewareVersion);
    deviceDescriptor.gatewareRevision = std::to_string(gatewareRevision);
    deviceDescriptor.hardwareVersion = std::to_string(hwVersion);

    return deviceDescriptor;
}

OpStatus LimeSDR::Reset()
{
    return LMS64CProtocol::DeviceReset(*mSerialPort, 0);
}

OpStatus LimeSDR::EnableChannel(uint8_t moduleIndex, TRXDir trx, uint8_t channel, bool enable)
{
    OpStatus status = mLMSChips.at(moduleIndex)->EnableChannel(trx, channel, enable);
    if (trx == TRXDir::Tx) //always enable DAC1, otherwise sample rates <2.5MHz do not work
        mLMSChips[0]->Modify_SPI_Reg_bits(PD_TX_AFE1, 0);
    return status;
}

double LimeSDR::GetClockFreq(uint8_t clk_id, uint8_t channel)
{
    return mLMSChips[0]->GetClockFreq(static_cast<LMS7002M::ClockID>(clk_id));
}

OpStatus LimeSDR::SetClockFreq(uint8_t clk_id, double freq, uint8_t channel)
{
    return mLMSChips[0]->SetClockFreq(static_cast<LMS7002M::ClockID>(clk_id), freq);
}

OpStatus LimeSDR::SPI(uint32_t chipSelect, const uint32_t* MOSI, uint32_t* MISO, uint32_t count)
{
    switch (chipSelect)
    {
    case limesdrusb::SPI_LMS7002M:
        return mlms7002mPort->SPI(0, MOSI, MISO, count);
    case limesdrusb::SPI_FPGA:
        return mfpgaPort->SPI(MOSI, MISO, count);
    case limesdrusb::SPI_ADF4002:
        return LMS64CProtocol::ADF4002_SPI(*mSerialPort, MOSI, count);
    default:
        throw std::logic_error("LimeSDR SPI invalid SPI chip select"s);
    }
}

// There might be some leftover samples data still buffered in USB device
// clear the USB buffers before streaming samples to avoid old data
void LimeSDR::ResetUSBFIFO()
{
    lime::debug("LimeSDR: resetting USB FIFO");
    // Don't reset USB FIFO if stream is running, otherwise data will stop.
    LMS64CPacket pkt;
    pkt.cmd = Command::USB_FIFO_RST;
    pkt.status = CommandStatus::Undefined;
    pkt.blockCount = 1;
    pkt.payload[0] = 0;

    int sentBytes = mSerialPort->Write(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt), 100);
    if (sentBytes != sizeof(pkt))
    {
        throw std::runtime_error("LimeSDR::ResetUSBFIFO write failed"s);
    }

    int gotBytes = mSerialPort->Read(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt), 100);
    if (gotBytes != sizeof(pkt))
    {
        throw std::runtime_error("LimeSDR::ResetUSBFIFO read failed"s);
    }
}

OpStatus LimeSDR::GPIODirRead(uint8_t* buffer, const size_t bufLength)
{
    return mfpgaPort->GPIODirRead(buffer, bufLength);
}

OpStatus LimeSDR::GPIORead(uint8_t* buffer, const size_t bufLength)
{
    return mfpgaPort->GPIORead(buffer, bufLength);
}

OpStatus LimeSDR::GPIODirWrite(const uint8_t* buffer, const size_t bufLength)
{
    return mfpgaPort->GPIODirWrite(buffer, bufLength);
}

OpStatus LimeSDR::GPIOWrite(const uint8_t* buffer, const size_t bufLength)
{
    return mfpgaPort->GPIOWrite(buffer, bufLength);
}

OpStatus LimeSDR::CustomParameterWrite(const std::vector<CustomParameterIO>& parameters)
{
    return mfpgaPort->CustomParameterWrite(parameters);
}

OpStatus LimeSDR::CustomParameterRead(std::vector<CustomParameterIO>& parameters)
{
    return mfpgaPort->CustomParameterRead(parameters);
}

OpStatus LimeSDR::UploadMemory(
    eMemoryDevice device, uint8_t moduleIndex, const char* data, size_t length, UploadMemoryCallback callback)
{
    int progMode;
    LMS64CProtocol::ALTERA_FPGA_GW_WR_targets target = LMS64CProtocol::ALTERA_FPGA_GW_WR_targets::FPGA;

    // TODO: add FX3 firmware flashing
    switch (device)
    {
    case eMemoryDevice::FPGA_RAM:
        progMode = 0;
        break;
    case eMemoryDevice::FPGA_FLASH:
        progMode = 1;
        break;
    default:
        return OpStatus::InvalidValue;
    }

    return mfpgaPort->ProgramWrite(data, length, progMode, static_cast<int>(target), callback);
}

OpStatus LimeSDR::MemoryWrite(std::shared_ptr<DataStorage> storage, Region region, const void* data)
{
    if (storage == nullptr || storage->ownerDevice != this || storage->memoryDeviceType != eMemoryDevice::EEPROM)
        return OpStatus::Error;
    return mfpgaPort->MemoryWrite(region.address, data, region.size);
}

OpStatus LimeSDR::MemoryRead(std::shared_ptr<DataStorage> storage, Region region, void* data)
{
    if (storage == nullptr || storage->ownerDevice != this || storage->memoryDeviceType != eMemoryDevice::EEPROM)
        return OpStatus::Error;
    return mfpgaPort->MemoryRead(region.address, data, region.size);
}

std::unique_ptr<lime::RFStream> LimeSDR::StreamCreate(const StreamConfig& config, uint8_t moduleIndex)
{
    constexpr uint8_t rxBulkEndpoint = 0x81;
    constexpr uint8_t txBulkEndpoint = 0x01;
    auto rxdma = std::make_shared<USBDMAEmulation>(mStreamPort, rxBulkEndpoint, DataTransferDirection::DeviceToHost);
    auto txdma = std::make_shared<USBDMAEmulation>(mStreamPort, txBulkEndpoint, DataTransferDirection::HostToDevice);

    ResetUSBFIFO();
    std::unique_ptr<TRXLooper> streamer = std::make_unique<TRXLooper>(
        std::static_pointer_cast<IDMA>(rxdma), std::static_pointer_cast<IDMA>(txdma), mFPGA.get(), mLMSChips.at(0).get(), 0);
    if (!streamer)
        return streamer;

    if (mCallback_logMessage)
        streamer->SetMessageLogCallback(mCallback_logMessage);
    OpStatus status = streamer->Setup(config);
    if (status != OpStatus::Success)
        return std::unique_ptr<RFStream>(nullptr);
    return streamer;
}

} // namespace lime
