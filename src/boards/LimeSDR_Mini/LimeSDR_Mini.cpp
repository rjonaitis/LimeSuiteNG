#include "LimeSDR_Mini.h"

#include "USBGeneric.h"
#include "LMSBoards.h"
#include "limesuiteng/LMS7002M.h"
#include "chips/Si5351C/Si5351C.h"
#include "LMS64CProtocol.h"
#include "limesuiteng/Logger.h"
#include "FPGA_Mini.h"
#include "comms/USB/USBDMA.h"
#include "chips/LMS7002M/validation.h"
#include "chips/LMS7002M/LMS7002MCSR_Data.h"
#include "DSP/Equalizer/EqualizerCSR.h"
#include "protocols/LMS64CProtocol.h"
#include "DeviceTreeNode.h"
#include "comms/IComms.h"
#include "ISerialPort.h"
#include "FT601/FT601.h"

#include <assert.h>
#include <memory>
#include <set>
#include <stdexcept>
#include <cmath>

#ifdef __unix__
    #include "libusb.h"
#endif

using namespace lime;
using namespace lime::LMS64CProtocol;
using namespace lime::EqualizerCSR;
using namespace lime::LMS7002MCSR_Data;
using namespace std::literals::string_literals;

static const int STREAM_BULK_WRITE_ADDRESS = 0x03;
static const int STREAM_BULK_READ_ADDRESS = 0x83;

static const uint8_t SPI_LMS7002M = 0;
static const uint8_t SPI_FPGA = 1;

static const CustomParameter CP_VCTCXO_DAC = { "VCTCXO DAC (runtime)"s, 0, 0, 255, false };
static const CustomParameter CP_TEMPERATURE = { "Board Temperature"s, 1, 0, 65535, true };

static const std::vector<std::pair<uint16_t, uint16_t>> lms7002defaultsOverrides_1v0 = { //
    { 0x0022, 0x0FFF },
    { 0x0023, 0x5550 },
    { 0x002B, 0x0038 },
    { 0x002C, 0x0000 },
    { 0x002D, 0x0641 },
    { 0x0086, 0x4101 },
    { 0x0087, 0x5555 },
    { 0x0088, 0x03F0 },
    { 0x0089, 0x1078 },
    { 0x008B, 0x2100 },
    { 0x008C, 0x267B },
    { 0x0092, 0xFFFF },
    { 0x0093, 0x03FF },
    { 0x00A1, 0x656A },
    { 0x00A6, 0x0001 },
    { 0x00A9, 0x8000 },
    { 0x00AC, 0x2000 },
    { 0x0105, 0x0011 },
    { 0x0108, 0x218C },
    { 0x0109, 0x6100 },
    { 0x010A, 0x1F4C },
    { 0x010B, 0x0001 },
    { 0x010C, 0x8865 },
    { 0x010E, 0x0000 },
    { 0x010F, 0x3142 },
    { 0x0110, 0x2B14 },
    { 0x0111, 0x0000 },
    { 0x0112, 0x942E },
    { 0x0113, 0x03C2 },
    { 0x0114, 0x00D0 },
    { 0x0117, 0x1230 },
    { 0x0119, 0x18D2 },
    { 0x011C, 0x8941 },
    { 0x011D, 0x0000 },
    { 0x011E, 0x0740 },
    { 0x0120, 0xE6C0 },
    { 0x0121, 0x8650 },
    { 0x0123, 0x000F },
    { 0x0200, 0x00E1 },
    { 0x0208, 0x017B },
    { 0x020B, 0x4000 },
    { 0x020C, 0x8000 },
    { 0x0400, 0x8081 },
    { 0x0404, 0x0006 },
    { 0x040B, 0x1020 },
    { 0x040C, 0x00FB }
};

static const std::vector<std::pair<uint16_t, uint16_t>> lms7002defaultsOverrides_1v2 = { //
    { 0x0022, 0x0FFF },
    { 0x0023, 0x5550 },
    { 0x002B, 0x0038 },
    { 0x002C, 0x0000 },
    { 0x002D, 0x0641 },
    { 0x0086, 0x4101 },
    { 0x0087, 0x5555 },
    { 0x0088, 0x03F0 },
    { 0x0089, 0x1078 },
    { 0x008B, 0x2100 },
    { 0x008C, 0x267B },
    { 0x00A1, 0x656A },
    { 0x00A6, 0x0009 },
    { 0x00A7, 0x8A8A },
    { 0x00A9, 0x8000 },
    { 0x00AC, 0x2000 },
    { 0x0105, 0x0011 },
    { 0x0108, 0x218C },
    { 0x0109, 0x6100 },
    { 0x010A, 0x1F4C },
    { 0x010B, 0x0001 },
    { 0x010C, 0x8865 },
    { 0x010E, 0x0000 },
    { 0x010F, 0x3142 },
    { 0x0110, 0x2B14 },
    { 0x0111, 0x0000 },
    { 0x0112, 0x942E },
    { 0x0113, 0x03C2 },
    { 0x0114, 0x00D0 },
    { 0x0117, 0x1230 },
    { 0x0119, 0x18D2 },
    { 0x011C, 0x8941 },
    { 0x011D, 0x0000 },
    { 0x011E, 0x0740 },
    { 0x0120, 0xC5C0 },
    { 0x0121, 0x8650 },
    { 0x0123, 0x000F },
    { 0x0200, 0x00E1 },
    { 0x0208, 0x017B },
    { 0x020B, 0x4000 },
    { 0x020C, 0x8000 },
    { 0x0400, 0x8081 },
    { 0x0404, 0x0006 },
    { 0x040B, 0x1020 },
    { 0x040C, 0x00FB }
};

/// @brief Constructs a new LimeSDR_Mini object
/// @param spiLMS The communications port to the LMS7002M chip.
/// @param spiFPGA The communications port to the device's FPGA.
/// @param streamPort The communications port to send and receive sample data.
/// @param commsPort The communications port for direct communications with the device.
LimeSDR_Mini::LimeSDR_Mini(std::shared_ptr<IComms> spiLMS,
    std::shared_ptr<IComms> spiFPGA,
    std::shared_ptr<USBGeneric> streamPort,
    std::shared_ptr<ISerialPort> commsPort)
    : mStreamPort(streamPort)
    , mSerialPort(commsPort)
    , mlms7002mPort(spiLMS)
    , mfpgaPort(spiFPGA)
{
    SDRDescriptor descriptor = GetDeviceInfo();

    LMS7002M* chip = new LMS7002M(mlms7002mPort);
    chip->SetConnection(mlms7002mPort);
    chip->SetOnCGENChangeCallback(UpdateFPGAInterface, this);
    mLMSChips.push_back(chip);

    mFPGA = new FPGA_Mini(spiFPGA, spiLMS);
    double refClk = mFPGA->DetectRefClk();
    chip->SetReferenceClk_SX(TRXDir::Rx, refClk);

    FPGA::GatewareInfo gw = mFPGA->GetGatewareInfo();
    FPGA::GatewareToDescriptor(gw, descriptor);

    if (gw.hardwareVersion >= 2)
        chip->ModifyRegistersDefaults(lms7002defaultsOverrides_1v2);
    else
        chip->ModifyRegistersDefaults(lms7002defaultsOverrides_1v0);

    mStreamers.resize(1, nullptr);

    descriptor.customParameters.push_back(CP_VCTCXO_DAC);

    if (descriptor.name == GetDeviceName(LMS_DEV_LIMESDRMINI_V2))
    {
        descriptor.customParameters.push_back(CP_TEMPERATURE);
    }

    descriptor.spiSlaveIds = { { "LMS7002M"s, SPI_LMS7002M }, { "FPGA"s, SPI_FPGA } };

    RFSOCDescriptor soc;
    soc.name = "LMS";
    soc.channelCount = 1;
    soc.pathNames[TRXDir::Rx] = { "NONE"s, "LNAH"s, "LNAL_NC"s, "LNAW"s, "Auto"s };
    soc.pathNames[TRXDir::Tx] = { "NONE"s, "Band1"s, "Band2"s, "Auto"s };
    soc.samplingRateRange = { 100e3, 30.72e6, 0 };
    soc.frequencyRange = { 10e6, 3.5e9, 0 };

    soc.lowPassFilterRange[TRXDir::Rx] = { 1.4001e6, 130e6 };
    soc.lowPassFilterRange[TRXDir::Tx] = { 5e6, 130e6 };

    soc.antennaRange[TRXDir::Rx]["LNAH"s] = { 2e9, 2.6e9 };
    soc.antennaRange[TRXDir::Rx]["LNAW"s] = { 700e6, 900e6 };
    soc.antennaRange[TRXDir::Tx]["Band1"s] = { 2e9, 2.6e9 };
    soc.antennaRange[TRXDir::Tx]["Band2"s] = { 30e6, 1.9e9 };

    SetGainInformationInDescriptor(soc);

    descriptor.rfSOC.push_back(soc);

    auto fpgaNode = std::make_shared<DeviceTreeNode>("FPGA"s, eDeviceTreeNodeClass::FPGA_MINI, mFPGA);
    fpgaNode->children.push_back(std::make_shared<DeviceTreeNode>("LMS"s, eDeviceTreeNodeClass::LMS7002M, mLMSChips[0]));
    descriptor.socTree = std::make_shared<DeviceTreeNode>("SDR Mini"s, eDeviceTreeNodeClass::SDRDevice, this);
    descriptor.socTree->children.push_back(fpgaNode);

    mDeviceDescriptor = descriptor;
}

LimeSDR_Mini::~LimeSDR_Mini()
{
    auto& streamer = mStreamers.at(0);
    if (streamer != nullptr && streamer->IsStreamRunning())
    {
        streamer->Stop();
    }
}

OpStatus LimeSDR_Mini::Configure(const SDRConfig& cfg, uint8_t moduleIndex = 0)
{
    OpStatus status = OpStatus::Success;
    std::vector<std::string> errors;
    bool isValidConfig = LMS7002M_Validate(cfg, errors, 1);

    if (!isValidConfig)
    {
        std::stringstream ss;

        for (const auto& err : errors)
        {
            ss << err << std::endl;
        }

        return lime::ReportError(OpStatus::Error, "LimeSDR-Mini: "s + ss.str());
    }

    bool rxUsed = false;
    bool txUsed = false;
    for (int i = 0; i < 2; ++i)
    {
        const ChannelConfig& ch = cfg.channel[i];
        rxUsed |= ch.rx.enabled;
        txUsed |= ch.tx.enabled;
    }

    // config validation complete, now do the actual configuration
    try
    {
        mConfigInProgress = true;
        LMS7002M* chip = mLMSChips.at(0);
        if (!cfg.skipDefaults)
        {
            status = Init();
            if (status != OpStatus::Success)
                return status;
        }

        status = LMS7002LOConfigure(chip, cfg);
        if (status != OpStatus::Success)
            return lime::ReportError(OpStatus::Error, "LimeSDR_Mini: LO configuration failed."s);
        for (int i = 0; i < 2; ++i)
        {
            status = LMS7002ChannelConfigure(chip, cfg.channel[i], i);
            if (status != OpStatus::Success)
                return lime::ReportError(OpStatus::Error, "LimeSDR_Mini: channel%i configuration failed.", i);
            LMS7002TestSignalConfigure(chip, cfg.channel[i], i);
        }

        // enabled ADC/DAC is required for FPGA to work
        chip->Modify_SPI_Reg_bits(PD_RX_AFE1, 0);
        chip->Modify_SPI_Reg_bits(PD_TX_AFE1, 0);
        chip->SetActiveChannel(LMS7002M::Channel::ChA);

        double sampleRate;
        if (rxUsed)
            sampleRate = cfg.channel[0].rx.sampleRate;
        else
            sampleRate = cfg.channel[0].tx.sampleRate;
        if (sampleRate > 0)
        {
            status = SetSampleRate(0, TRXDir::Rx, 0, sampleRate, cfg.channel[0].rx.oversample);
            if (status != OpStatus::Success)
                return lime::ReportError(OpStatus::Error, "LimeSDR_Mini: failed to set sampling rate."s);
        }

        for (int i = 0; i < 2; ++i)
        {
            const ChannelConfig& ch = cfg.channel[i];
            LMS7002ChannelCalibration(chip, ch, i);
            // TODO: should report calibration failure, but configuration can
            // still work after failed calibration.
        }
        chip->SetActiveChannel(LMS7002M::Channel::ChA);

        mConfigInProgress = false;
        if (sampleRate > 0)
        {
            status = UpdateFPGAInterface(this);
            if (status != OpStatus::Success)
                return lime::ReportError(OpStatus::Error, "LimeSDR_Mini: failed to update FPGA interface frequency."s);
        }
    } //try
    catch (std::logic_error& e)
    {
        return ReportError(OpStatus::Error, "LimeSDR_Mini config: "s + e.what());
    } catch (std::runtime_error& e)
    {
        return ReportError(OpStatus::Error, "LimeSDR_Mini config: "s + e.what());
    }
    return OpStatus::Success;
}

OpStatus LimeSDR_Mini::Init()
{
    lime::LMS7002M* lms = mLMSChips[0];
    OpStatus status;
    status = lms->ResetChip();
    if (status != OpStatus::Success)
        return status;

    lms->Modify_SPI_Reg_bits(LMS7002MCSR::MAC, 1);

    if (lms->CalibrateTxGain() != OpStatus::Success)
        return OpStatus::Error;

    lms->EnableChannel(TRXDir::Tx, 0, false);

    lms->Modify_SPI_Reg_bits(LMS7002MCSR::MAC, 2);
    lms->SPI_write(0x0123, 0x000F); //SXT
    lms->SPI_write(0x0120, 0x80C0); //SXT
    lms->SPI_write(0x011C, 0x8941); //SXT
    lms->EnableChannel(TRXDir::Rx, 0, false);
    lms->EnableChannel(TRXDir::Tx, 0, false);

    lms->Modify_SPI_Reg_bits(LMS7002MCSR::MAC, 1);

    /*bool auto_path[2] = {auto_tx_path, auto_rx_path};
    auto_tx_path = false;
    auto_rx_path = false;

    if(SetFrequency(true, 0, GetFrequency(true, 0)) != 0)
    {
        return -1;
    }

    if(SetFrequency(false, 0, GetFrequency(false, 0)) != 0)
    {
        return -1;
    }

    auto_tx_path = auto_path[0];
    auto_rx_path = auto_path[1];

    if (SetRate(15.36e6, 1) != 0)
    {
        return -1;
    }*/

    return OpStatus::Success;
}

OpStatus LimeSDR_Mini::Reset()
{
    return LMS64CProtocol::DeviceReset(*mSerialPort, 0);
}

double LimeSDR_Mini::GetClockFreq(uint8_t clk_id, uint8_t channel)
{
    return mLMSChips[0]->GetClockFreq(static_cast<LMS7002M::ClockID>(clk_id));
}

OpStatus LimeSDR_Mini::SetClockFreq(uint8_t clk_id, double freq, uint8_t channel)
{
    return mLMSChips[0]->SetClockFreq(static_cast<LMS7002M::ClockID>(clk_id), freq);
}

OpStatus LimeSDR_Mini::Synchronize(bool toChip)
{
    if (toChip)
    {
        OpStatus status = mLMSChips[0]->UploadAll();
        if (status == OpStatus::Success)
        {
            //ret = SetFPGAInterfaceFreq(-1, -1, -1000, -1000); // TODO: implement
        }
        return status;
    }
    else
        return mLMSChips[0]->DownloadAll();
}

OpStatus LimeSDR_Mini::SPI(uint32_t chipSelect, const uint32_t* MOSI, uint32_t* MISO, uint32_t count)
{
    assert(mStreamPort);
    assert(MOSI);
    LMS64CPacket pkt;

    size_t srcIndex = 0;
    size_t destIndex = 0;
    constexpr int maxBlocks = LMS64CPacket::payloadSize / (sizeof(uint32_t) / sizeof(uint8_t)); // = 14

    while (srcIndex < count)
    {
        pkt.status = CommandStatus::Undefined;
        pkt.blockCount = 0;
        pkt.periphID = chipSelect;

        // fill packet with same direction operations
        const bool willDoWrite = MOSI[srcIndex] & (1 << 31);

        for (int i = 0; i < maxBlocks && srcIndex < count; ++i)
        {
            const bool isWrite = MOSI[srcIndex] & (1 << 31);

            if (isWrite != willDoWrite)
            {
                break; // change between write/read, flush packet
            }

            if (isWrite)
            {
                switch (chipSelect)
                {
                case SPI_LMS7002M:
                    pkt.cmd = Command::LMS7002_WR;
                    break;
                case SPI_FPGA:
                    pkt.cmd = Command::BRDSPI_WR;
                    break;
                default:
                    throw std::logic_error("LimeSDR SPI invalid SPI chip select"s);
                }

                int payloadOffset = pkt.blockCount * 4;
                pkt.payload[payloadOffset + 0] = MOSI[srcIndex] >> 24;
                pkt.payload[payloadOffset + 1] = MOSI[srcIndex] >> 16;
                pkt.payload[payloadOffset + 2] = MOSI[srcIndex] >> 8;
                pkt.payload[payloadOffset + 3] = MOSI[srcIndex];
            }
            else
            {
                switch (chipSelect)
                {
                case SPI_LMS7002M:
                    pkt.cmd = Command::LMS7002_RD;
                    break;
                case SPI_FPGA:
                    pkt.cmd = Command::BRDSPI_RD;
                    break;
                default:
                    throw std::logic_error("LimeSDR SPI invalid SPI chip select"s);
                }

                int payloadOffset = pkt.blockCount * 2;
                pkt.payload[payloadOffset + 0] = MOSI[srcIndex] >> 8;
                pkt.payload[payloadOffset + 1] = MOSI[srcIndex];
            }

            ++pkt.blockCount;
            ++srcIndex;
        }

        // flush packet
        //printPacket(pkt, 4, "Wr:");
        int sent = mSerialPort->Write(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt), 100);
        if (sent != sizeof(pkt))
        {
            throw std::runtime_error("SPI failed"s);
        }

        int recv = mSerialPort->Read(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt), 100);
        //printPacket(pkt, 4, "Rd:");

        if (recv >= pkt.headerSize + 4 * pkt.blockCount && pkt.status == CommandStatus::Completed)
        {
            for (int i = 0; MISO && i < pkt.blockCount && destIndex < count; ++i)
            {
                //MISO[destIndex] = 0;
                //MISO[destIndex] = pkt.payload[0] << 24;
                //MISO[destIndex] |= pkt.payload[1] << 16;
                MISO[destIndex] = (pkt.payload[i * 4 + 2] << 8) | pkt.payload[i * 4 + 3];
                ++destIndex;
            }
        }
        else
        {
            throw std::runtime_error("SPI failed"s);
        }
    }

    return OpStatus::Success;
}

// Callback for updating FPGA's interface clocks when LMS7002M CGEN is manually modified
OpStatus LimeSDR_Mini::UpdateFPGAInterface(void* userData)
{
    constexpr int chipIndex = 0;
    assert(userData != nullptr);
    LimeSDR_Mini* pthis = static_cast<LimeSDR_Mini*>(userData);
    LMS7002M* soc = pthis->mLMSChips[chipIndex];
    return UpdateFPGAInterfaceFrequency(*soc, *pthis->mFPGA, chipIndex);
}

double LimeSDR_Mini::GetTemperature(uint8_t moduleIndex)
{
    if (mDeviceDescriptor.name == GetDeviceName(LMS_DEV_LIMESDRMINI))
    {
        throw std::logic_error("LimeSDR-Mini v1 doesn't have a temperature sensor"s);
    }

    return LMS7002M_SDRDevice::GetTemperature(moduleIndex);
}

OpStatus LimeSDR_Mini::SetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double sampleRate, uint8_t oversample)
{
    const bool bypass = (oversample <= 1);
    uint8_t decimation = 7; // HBD_OVR_RXTSP=7 - bypass
    uint8_t interpolation = 7; // HBI_OVR_TXTSP=7 - bypass
    double cgenFreq = sampleRate * 4; // AI AQ BI BQ
    // TODO:
    // for (uint8_t i = 0; i < GetNumChannels(false) ;i++)
    // {
    //     if (rx_channels[i].cF_offset_nco != 0.0 || tx_channels[i].cF_offset_nco != 0.0)
    //     {
    //         bypass = false;
    //         break;
    //     }
    // }

    if (bypass)
    {
        lime::info("Sampling rate set(%.3f MHz): CGEN:%.3f MHz, Decim: bypass, Interp: bypass", sampleRate / 1e6, cgenFreq / 1e6);
    }
    else
    {
        if (oversample == 0)
        {
            const int n = lime::LMS7002M::CGEN_MAX_FREQ / (cgenFreq);
            oversample = (n >= 32) ? 32 : (n >= 16) ? 16 : (n >= 8) ? 8 : (n >= 4) ? 4 : 2;
        }

        decimation = 4;
        if (oversample <= 16)
        {
            constexpr std::array<int, 17> decimationTable{ 0, 0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3 };
            decimation = decimationTable.at(oversample);
        }
        interpolation = decimation;
        cgenFreq *= 2 << decimation;

        lime::info("Sampling rate set(%.3f MHz): CGEN:%.3f MHz, Decim: 2^%i, Interp: 2^%i",
            sampleRate / 1e6,
            cgenFreq / 1e6,
            decimation + 1,
            interpolation + 1); // dec/inter ratio is 2^(value+1)
    }

    mLMSChips.at(moduleIndex)->Modify_SPI_Reg_bits(LMS7002MCSR_Data::MAC, 1);
    mLMSChips.at(moduleIndex)->Modify_SPI_Reg_bits(LMS7002MCSR_Data::LML1_SISODDR, 1);
    mLMSChips.at(moduleIndex)->Modify_SPI_Reg_bits(LMS7002MCSR_Data::LML2_SISODDR, 1);
    mLMSChips.at(moduleIndex)->Modify_SPI_Reg_bits(LMS7002MCSR_Data::CDSN_RXALML, !bypass);
    mLMSChips.at(moduleIndex)->Modify_SPI_Reg_bits(LMS7002MCSR_Data::EN_ADCCLKH_CLKGN, 0);
    mLMSChips.at(moduleIndex)->Modify_SPI_Reg_bits(LMS7002MCSR_Data::CLKH_OV_CLKL_CGEN, 2);
    mLMSChips.at(moduleIndex)->Modify_SPI_Reg_bits(LMS7002MCSR_Data::MAC, 2);
    mLMSChips.at(moduleIndex)->Modify_SPI_Reg_bits(LMS7002MCSR_Data::HBD_OVR_RXTSP, decimation);
    mLMSChips.at(moduleIndex)->Modify_SPI_Reg_bits(LMS7002MCSR_Data::HBI_OVR_TXTSP, interpolation);
    mLMSChips.at(moduleIndex)->Modify_SPI_Reg_bits(LMS7002MCSR_Data::MAC, 1);
    if (bypass)
    {
        return mLMSChips.at(moduleIndex)->SetInterfaceFrequency(sampleRate * 4, 7, 7);
    }
    else
    {
        return mLMSChips.at(moduleIndex)->SetInterfaceFrequency(cgenFreq, interpolation, decimation);
    }
}

SDRDescriptor LimeSDR_Mini::GetDeviceInfo(void)
{
    assert(mStreamPort);
    SDRDescriptor deviceDescriptor;

    LMS64CProtocol::FirmwareInfo info;
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
    SPI(SPI_FPGA, addrs, data, 4);
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

OpStatus LimeSDR_Mini::StreamSetup(const StreamConfig& config, uint8_t moduleIndex)
{
    // Allow multiple setup calls
    if (mStreamers.at(0) != nullptr)
    {
        delete mStreamers.at(0);
    }

    auto connection = std::static_pointer_cast<FT601>(mStreamPort);
    connection->ResetStreamBuffers();

    mStreamers.at(0) = new TRXLooper(
        std::static_pointer_cast<IDMA>(std::make_shared<USBDMA>(mStreamPort, STREAM_BULK_READ_ADDRESS, STREAM_BULK_WRITE_ADDRESS)),
        mFPGA,
        mLMSChips.at(0),
        0);
    return mStreamers.at(0)->Setup(config);
}

void LimeSDR_Mini::StreamStart(uint8_t moduleIndex)
{
    if (mStreamers[0])
    {
        mStreamers[0]->Start();
    }
    else
    {
        throw std::runtime_error("Stream not setup"s);
    }
}

void LimeSDR_Mini::StreamStop(uint8_t moduleIndex)
{
    if (!mStreamers[0])
    {
        return;
    }

    mStreamers[0]->Stop();

    delete mStreamers[0];
    mStreamers[0] = nullptr;
}

OpStatus LimeSDR_Mini::GPIODirRead(uint8_t* buffer, const size_t bufLength)
{
    if (!buffer || bufLength == 0)
    {
        return OpStatus::InvalidValue;
    }

    const uint32_t addr = 0xC4;
    uint32_t value;

    OpStatus ret = mFPGA->ReadRegisters(&addr, &value, 1);
    buffer[0] = value;

    if (bufLength > 1)
    {
        buffer[1] = (value >> 8);
    }

    return ret;
}

OpStatus LimeSDR_Mini::GPIORead(uint8_t* buffer, const size_t bufLength)
{
    if (!buffer || bufLength == 0)
    {
        return OpStatus::InvalidValue;
    }

    const uint32_t addr = 0xC2;
    uint32_t value;

    OpStatus ret = mFPGA->ReadRegisters(&addr, &value, 1);
    buffer[0] = value;

    if (bufLength > 1)
    {
        buffer[1] = (value >> 8);
    }

    return ret;
}

OpStatus LimeSDR_Mini::GPIODirWrite(const uint8_t* buffer, const size_t bufLength)
{
    if (!buffer || bufLength == 0)
    {
        return OpStatus::InvalidValue;
    }

    const uint32_t addr = 0xC4;
    const uint32_t value = (bufLength == 1) ? buffer[0] : buffer[0] | (buffer[1] << 8);

    return mFPGA->WriteRegisters(&addr, &value, 1);
}

OpStatus LimeSDR_Mini::GPIOWrite(const uint8_t* buffer, const size_t bufLength)
{
    if (!buffer || bufLength == 0)
    {
        return OpStatus::InvalidValue;
    }

    const uint32_t addr = 0xC6;
    const uint32_t value = (bufLength == 1) ? buffer[0] : buffer[0] | (buffer[1] << 8);

    return mFPGA->WriteRegisters(&addr, &value, 1);
}

OpStatus LimeSDR_Mini::CustomParameterWrite(const std::vector<CustomParameterIO>& parameters)
{
    return mfpgaPort->CustomParameterWrite(parameters);
}

OpStatus LimeSDR_Mini::CustomParameterRead(std::vector<CustomParameterIO>& parameters)
{
    return mfpgaPort->CustomParameterRead(parameters);
}
