#include "LimeSDR_XTRX.h"

#include "limesuiteng/Logger.h"
#include "limesuiteng/LMS7002M.h"

#include <cmath>
#include <unistd.h>
#include <fcntl.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include "chips/LMS7002M/validation.h"
#include "chips/LMS7002M/LMS7002MCSR_Data.h"
#include "comms/IComms.h"
#include "comms/PCIe/LimePCIe.h"
#include "comms/PCIe/LimePCIeDMA.h"
#include "FPGA/FPGA_common.h"
#include "FPGA_XTRX.h"
#include "protocols/LMS64CProtocol.h"
#include "streaming/TRXLooper.h"
#include "utilities/toString.h"

#include "CommonFunctions.h"
#include "DeviceTreeNode.h"
#include "OEMTesting.h"

using namespace std::literals::string_literals;
using namespace lime::LMS7002MCSR_Data;

namespace lime {

namespace limesdrxtrx {
// XTRX board specific devices ids and data
static const uint8_t SPI_LMS7002M = 0;
static const uint8_t SPI_FPGA = 1;

static CustomParameter cp_vctcxo_dac = { "VCTCXO DAC (volatile)"s, 0, 0, 65535, false };
static const CustomParameter cp_temperature = { "Board Temperature"s, 1, 0, 65535, true };

// Fairwaves XTRX rev.5 requires specific LDO configuration to work properly
static const std::vector<std::pair<uint16_t, uint16_t>> lms7002defaultsOverrides_fairwaves_xtrx_rev5 = {
    { 0x0022, 0x0FFF },
    { 0x0023, 0x5550 },
    { 0x002B, 0x0038 },
    { 0x002C, 0x0000 },
    { 0x008B, 0x218C },
    { 0x00A6, 0x000F },
    { 0x011C, 0x8941 },
    { 0x0120, 0xE6C0 },
    { 0x0121, 0x3638 },
    { 0x0122, 0x0514 },
    { 0x0123, 0x200F },
    // LDOs
    { 0x0092, 0x0D15 },
    { 0x0093, 0x01B1 },
    { 0x00A6, 0x000F },
    // XBUF
    { 0x0085, 0x0019 },
};

static const std::vector<std::pair<uint16_t, uint16_t>> lms7002defaultsOverrides_limesdr_xtrx = {
    { 0x0020, 0xFFFD },
    { 0x0023, 0x5550 },
    { 0x002B, 0x0038 },
    { 0x002C, 0x0000 },
    { 0x0081, 0x0001 },
    { 0x0086, 0x4101 },
    { 0x0089, 0x1040 },
    { 0x008B, 0x2198 },
    { 0x009B, 0x8C65 },
    { 0x009E, 0x8C65 },
    { 0x00A0, 0x658C },
    { 0x00A6, 0x000F },
    { 0x0100, 0x7409 },
    { 0x0101, 0x1800 },
    { 0x0103, 0x0A50 },
    { 0x0105, 0x0011 },
    { 0x0108, 0x410C },
    { 0x010A, 0x1FFF },
    { 0x010B, 0x0001 },
    { 0x010C, 0x8865 },
    { 0x010D, 0x009F },
    { 0x010F, 0x3042 },
    { 0x0110, 0x2B14 },
    { 0x0111, 0x0000 },
    { 0x0112, 0x2106 },
    { 0x0113, 0x01C1 },
    { 0x0114, 0x01B0 },
    { 0x0117, 0x2044 },
    { 0x0119, 0x528C },
    { 0x011A, 0x3001 },
    { 0x011C, 0x8141 },
    { 0x011F, 0x3602 },
    { 0x0120, 0x35FF },
    { 0x0121, 0x37F8 },
    { 0x0122, 0x0654 },
    { 0x0124, 0x001F },
    { 0x0208, 0x017B },
    { 0x0400, 0x8081 },
    { 0x0405, 0x0303 },
    { 0x0406, 0x0303 },
    { 0x0407, 0x0303 },
    { 0x040A, 0x2000 },
    { 0x040C, 0x01FF },
};

} // namespace limesdrxtrx

// Callback for updating FPGA's interface clocks when LMS7002M CGEN is manually modified
OpStatus LimeSDR_XTRX::LMS1_UpdateFPGAInterface(void* userData)
{
    assert(userData != nullptr);
    LimeSDR_XTRX* pthis = static_cast<LimeSDR_XTRX*>(userData);
    // don't care about cgen changes while doing Config(), to avoid unnecessary fpga updates
    if (pthis->mConfigInProgress)
        return OpStatus::Success;
    return UpdateFPGAInterfaceFrequency(*pthis->mLMSChips.at(0), *pthis->mFPGA, 0);
}

/// @brief Constructs a new LimeSDR_XTRX object
///
/// @param spiRFsoc The communications port to the LMS7002M chip.
/// @param spiFPGA The communications port to the device's FPGA.
/// @param sampleStream The communications port to send and receive sample data.
/// @param control The serial port communication of the device.
/// @param refClk The reference clock of the device.
LimeSDR_XTRX::LimeSDR_XTRX(std::shared_ptr<IComms> spiRFsoc,
    std::shared_ptr<IComms> spiFPGA,
    std::shared_ptr<LimePCIe> sampleStream,
    std::shared_ptr<ISerialPort> control,
    double refClk)
    : LMS7002M_SDRDevice()
    , lms7002mPort(spiRFsoc)
    , fpgaPort(spiFPGA)
    , mStreamPort(sampleStream)
    , mSerialPort(control)
    , mConfigInProgress(false)
{
    mStreamers.resize(1);
    /// Do not perform any unnecessary configuring to device in constructor, so you
    /// could read back it's state for debugging purposes.
    SDRDescriptor& desc = mDeviceDescriptor;
    desc.name = GetDeviceName(LMS_DEV_LIMESDR_XTRX);

    LMS64CProtocol::FirmwareInfo fw{};
    LMS64CProtocol::GetFirmwareInfo(*mSerialPort, fw);
    LMS64CProtocol::FirmwareToDescriptor(fw, desc);

    desc.spiSlaveIds = { { "LMS7002M"s, limesdrxtrx::SPI_LMS7002M }, { "FPGA"s, limesdrxtrx::SPI_FPGA } };

    // const std::unordered_map<std::string, Region> flashMap = { { "VCTCXO_DAC"s, { 0x01FF0000, 2 } } };
    desc.memoryDevices[ToString(eMemoryDevice::FPGA_FLASH)] = std::make_shared<DataStorage>(this, eMemoryDevice::FPGA_FLASH);

    {
        // VCTCXO_DAC is actually stored in FLASH 0x01FF0000, as XTRX does not have EEPROM,
        // but because firmware code does not allow to directly write/read all FLASH addresses,
        // VCTCXO_DAC has to be used through "fake" EEPROM commands

        const std::unordered_map<std::string, Region> eepromMap = { { "VCTCXO_DAC"s, { 0x0010, 2 } } };
        desc.memoryDevices[ToString(eMemoryDevice::EEPROM)] =
            std::make_shared<DataStorage>(this, eMemoryDevice::EEPROM, std::move(eepromMap));
    }

    desc.customParameters = { limesdrxtrx::cp_vctcxo_dac, limesdrxtrx::cp_temperature };

    mFPGA = std::make_unique<lime::FPGA_XTRX>(spiFPGA, spiRFsoc);
    FPGA::GatewareInfo gw = mFPGA->GetGatewareInfo();
    FPGA::GatewareToDescriptor(gw, desc);

    const bool isFairwavesRev5 = gw.hardwareVersion == 0;

    // Initial XTRX gateware supported only 32bit DMA, it worked fine on x86 with the PCIe driver
    // limiting the address mask to 32bit, but some systems require at least 35bits,
    // like Raspberry Pi, or other Arm systems. If host requires more than 32bit DMA mask
    // the driver starts using 64bit mask, in that case it's a matter of luck if the system
    // provided DMA addresses will be in 32bit zone, and could work, otherwise, data will be
    // seen as transferred, but the values will be undefined.
    // LimeSDR XTRX gateware added 64bit DMA support in 1.13
    // Fairwaves XTRX Rev 5 gateware added 64bit DMA support in 1.3
    if (gw.version == 1 && ((isFairwavesRev5 && gw.revision < 4) || (!isFairwavesRev5 && gw.revision < 13)))
    {
        lime::warning("Current XTRX gateware does not support 64bit DMA addressing. "
                      "RF data streaming might not work. "
                      "Please update gateware."s);
    }

    const bool isGoldGatewareActive = static_cast<uint16_t>(gw.version) == 0xDEAD && static_cast<uint16_t>(gw.revision) == 0xDEAD;
    if (isGoldGatewareActive)
        lime::warning("XTRX FPGA is running backup 'gold' image, 'user' image might be corrupted, and need reflashing");

    // LimeSDR XTRX gateware revision 1.13 introduced "dual boot" images
    if ((!isFairwavesRev5 && gw.version >= 1 && gw.revision >= 13) || isGoldGatewareActive)
    {
        desc.memoryDevices[ToString(eMemoryDevice::GATEWARE_GOLD_IMAGE)] =
            std::make_shared<DataStorage>(this, eMemoryDevice::GATEWARE_GOLD_IMAGE);
        desc.memoryDevices[ToString(eMemoryDevice::GATEWARE_USER_IMAGE)] =
            std::make_shared<DataStorage>(this, eMemoryDevice::GATEWARE_USER_IMAGE);
    }

    {
        RFSOCDescriptor soc = GetDefaultLMS7002MDescriptor();
        soc.antennaRange[TRXDir::Rx]["LNAH"s] = { 3.3e9, 3.8e9 };
        soc.antennaRange[TRXDir::Rx]["LNAL"s] = { 0.3e9, 2.2e9 };
        soc.antennaRange[TRXDir::Rx]["LNAW"s] = { 0.7e9, 2.6e9 };
        soc.antennaRange[TRXDir::Rx]["LB1"s] = soc.antennaRange[TRXDir::Rx]["LNAL"s];
        soc.antennaRange[TRXDir::Rx]["LB2"s] = soc.antennaRange[TRXDir::Rx]["LNAW"s];
        soc.antennaRange[TRXDir::Tx]["Band1"s] = { 3.3e9, 3.8e9 };
        soc.antennaRange[TRXDir::Tx]["Band2"s] = { 0.03e9, 1.9e9 };

        desc.rfSOC.push_back(soc);

        std::unique_ptr<LMS7002M> chip = std::make_unique<LMS7002M>(spiRFsoc);

        if (isFairwavesRev5)
            chip->ModifyRegistersDefaults(limesdrxtrx::lms7002defaultsOverrides_fairwaves_xtrx_rev5);
        else // LimeSDR XTRX
            chip->ModifyRegistersDefaults(limesdrxtrx::lms7002defaultsOverrides_limesdr_xtrx);
        chip->SetOnCGENChangeCallback(LMS1_UpdateFPGAInterface, this);
        chip->SetReferenceClk_SX(TRXDir::Rx, refClk);
        chip->SetClockFreq(LMS7002M::ClockID::CLK_REFERENCE, refClk);
        mLMSChips.push_back(std::move(chip));
    }

    auto fpgaNode = std::make_shared<DeviceTreeNode>("FPGA"s, eDeviceTreeNodeClass::FPGA_XTRX, mFPGA.get());
    fpgaNode->children.push_back(
        std::make_shared<DeviceTreeNode>("LMS7002M"s, eDeviceTreeNodeClass::LMS7002M, mLMSChips.at(0).get()));
    desc.socTree = std::make_shared<DeviceTreeNode>("XTRX"s, eDeviceTreeNodeClass::SDRDevice, this);
    desc.socTree->children.push_back(fpgaNode);
}

static OpStatus InitLMS1(LMS7002M& lms, bool skipTune = false)
{
    OpStatus status;
    status = lms.ResetChip();
    if (status != OpStatus::Success)
        return status;

    if (skipTune)
        return OpStatus::Success;

    status = lms.SetFrequencySX(TRXDir::Tx, lms.GetFrequencySX(TRXDir::Tx));
    if (status != OpStatus::Success)
        return status;

    status = lms.SetFrequencySX(TRXDir::Rx, lms.GetFrequencySX(TRXDir::Rx));
    if (status != OpStatus::Success)
        return status;

    // if (SetRate(10e6,2)!=0)
    //     return -1;
    return OpStatus::Success;
}

OpStatus LimeSDR_XTRX::Configure(const SDRConfig& cfg, uint8_t socIndex)
{
    auto& chip = mLMSChips.at(0);

    mConfigInProgress = true;
    if (!cfg.skipDefaults)
    {
        const bool skipTune = true;
        InitLMS1(*chip, skipTune);
    }

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
        LMS1_SetSampleRate(sampleRate, cfg.channel[0].rx.oversample, cfg.channel[0].tx.oversample);

    for (int c = 0; c < 2; ++c)
    {
        LMSSetPath(TRXDir::Tx, c, cfg.channel[c].tx.path);
        LMSSetPath(TRXDir::Rx, c, cfg.channel[c].rx.path);
        LMS7002ChannelCalibration(*chip, cfg.channel[c], c);
    }

    if (sampleRate > 0)
        return LMS1_UpdateFPGAInterface(this);

    return OpStatus::Success;
}

OpStatus LimeSDR_XTRX::Init()
{
    struct regVal {
        uint16_t adr;
        uint16_t val;
    };

    const std::vector<regVal> mFPGAInitVals = {
        { 0x00D1, 0x3357 }, // RF Switches
    };

    for (auto i : mFPGAInitVals)
        mFPGA->WriteRegister(i.adr, i.val);

    // uint8_t paramId = 2;
    // double dacVal = 65535;
    // CustomParameterWrite(&paramId,&dacVal,1,"");
    // paramId = 3;
    // CustomParameterWrite(&paramId,&dacVal,1,"");

    OpStatus status = LMS64CProtocol::DeviceReset(*mSerialPort, 0);
    // XTRX on X8 board don't have Reset command, returns Unknown
    if (status != OpStatus::Success && status != OpStatus::NotImplemented)
        return status;

    const bool skipTune = true;
    return InitLMS1(*mLMSChips.at(0), skipTune);
}

OpStatus LimeSDR_XTRX::SetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double sampleRate, uint8_t oversample)
{
    return LMS1_SetSampleRate(sampleRate, oversample, oversample);
}

double LimeSDR_XTRX::GetClockFreq(uint8_t clk_id, uint8_t channel)
{
    auto& chip = mLMSChips.at(channel / 2);
    return chip->GetClockFreq(static_cast<LMS7002M::ClockID>(clk_id));
}

OpStatus LimeSDR_XTRX::SetClockFreq(uint8_t clk_id, double freq, uint8_t channel)
{
    auto& chip = mLMSChips.at(channel / 2);
    return chip->SetClockFreq(static_cast<LMS7002M::ClockID>(clk_id), freq);
}

OpStatus LimeSDR_XTRX::SPI(uint32_t chipSelect, const uint32_t* MOSI, uint32_t* MISO, uint32_t count)
{
    switch (chipSelect)
    {
    case limesdrxtrx::SPI_LMS7002M:
        return lms7002mPort->SPI(MOSI, MISO, count);
    case limesdrxtrx::SPI_FPGA:
        return fpgaPort->SPI(MOSI, MISO, count);
    default:
        throw std::logic_error("invalid SPI chip select"s);
    }
}

OpStatus LimeSDR_XTRX::LMS1_SetSampleRate(double f_Hz, uint8_t rxDecimation, uint8_t txInterpolation)
{
    if (f_Hz <= 61.44e6)
    {
        if (rxDecimation == 1)
            rxDecimation = 2;
        if (txInterpolation == 1)
            txInterpolation = 2;
    }
    else // sample rate above 61.44MHz is supported only in SISO mode, and no oversampling
    {
        rxDecimation = 1;
        txInterpolation = 1;
    }

    if (f_Hz > 61.44e6)
    {
        auto& lms = mLMSChips.at(0);
        // LimeLight & Pad
        lms->Modify_SPI_Reg_bits(LMS7002MCSR::DIQ2_DS, 1);
        lms->Modify_SPI_Reg_bits(LMS7002MCSR::LML1_SISODDR, 1);
        lms->Modify_SPI_Reg_bits(LMS7002MCSR::LML2_SISODDR, 1);
        // CDS
        lms->Modify_SPI_Reg_bits(LMS7002MCSR::CDSN_RXALML, 0);
        lms->Modify_SPI_Reg_bits(LMS7002MCSR::CDS_RXALML, 1);
        // LDO
        lms->Modify_SPI_Reg_bits(LMS7002MCSR::PD_LDO_DIGIp1, 0);
        lms->Modify_SPI_Reg_bits(LMS7002MCSR::PD_LDO_DIGIp2, 0);
        lms->Modify_SPI_Reg_bits(LMS7002MCSR::RDIV_DIGIp2, 140);
    }

    return LMS7002M_SDRDevice::LMS7002M_SetSampleRate(f_Hz, rxDecimation, txInterpolation);
}

void LimeSDR_XTRX::LMSSetPath(TRXDir dir, uint8_t chan, uint8_t pathId)
{
    uint16_t sw_addr = 0x000A;
    uint16_t sw_val = mFPGA->ReadRegister(sw_addr);

    auto& lms = mLMSChips.at(0);
    LMS7002M::ChannelScope scope(lms.get(), chan);

    if (dir == TRXDir::Tx)
    {
        switch (ePathLMS1_Tx(pathId))
        {
        case ePathLMS1_Tx::NONE: // RF switch don't need to change. Still set value to be deterministic.
        case ePathLMS1_Tx::BAND1:
            sw_val |= 1 << 4;
            break;
        case ePathLMS1_Tx::BAND2:
            sw_val &= ~(1 << 4);
            break;
        default:
            lime::error("Invalid Tx RF path"s);
        }
        lms->SetBandTRF(pathId);
    }
    else
    {
        lime::LMS7002M::PathRFE path{ pathId };
        // first configure chip path or loopback
        lms->SetPathRFE(lime::LMS7002M::PathRFE(path));

        // configure rf switches ignoring loopback values
        if (path == LMS7002M::PathRFE::LB1)
            path = LMS7002M::PathRFE::LNAL;
        else if (path == LMS7002M::PathRFE::LB2)
            path = LMS7002M::PathRFE::LNAW;

        sw_val &= ~(0x3 << 2);
        if (path == LMS7002M::PathRFE::LNAW)
            sw_val &= ~(0x3 << 2);
        else if (path == LMS7002M::PathRFE::LNAH)
            sw_val |= 2 << 2;
        else if (path == LMS7002M::PathRFE::LNAL)
            sw_val |= 1 << 2;
    }
    // TODO: if MIMO use channel 0 as deciding factor, otherwise use active channel
    // RF switch controls are toggled for both channels, use channel 0 as the deciding source.
    if (chan == 0)
        mFPGA->WriteRegister(sw_addr, sw_val);
}

OpStatus LimeSDR_XTRX::CustomParameterWrite(const std::vector<CustomParameterIO>& parameters)
{
    return fpgaPort->CustomParameterWrite(parameters);
}

OpStatus LimeSDR_XTRX::CustomParameterRead(std::vector<CustomParameterIO>& parameters)
{
    return fpgaPort->CustomParameterRead(parameters);
}

OpStatus LimeSDR_XTRX::UploadMemory(
    eMemoryDevice device, uint8_t moduleIndex, const char* data, size_t length, UploadMemoryCallback callback)
{
    int progMode;
    LMS64CProtocol::ALTERA_FPGA_GW_WR_targets target = LMS64CProtocol::ALTERA_FPGA_GW_WR_targets::FPGA;

    switch (device)
    {
    case eMemoryDevice::FPGA_RAM:
        progMode = 0;
        break;
    case eMemoryDevice::FPGA_FLASH:
        progMode = 1;
        break;
    case eMemoryDevice::GATEWARE_GOLD_IMAGE:
        progMode = 3;
        break;
    case eMemoryDevice::GATEWARE_USER_IMAGE:
        progMode = 4;
        break;
    default:
        return OpStatus::InvalidValue;
    }

    return fpgaPort->ProgramWrite(data, length, progMode, static_cast<int>(target), callback);
}

OpStatus LimeSDR_XTRX::MemoryWrite(std::shared_ptr<DataStorage> storage, Region region, const void* data)
{
    if (storage == nullptr || storage->ownerDevice != this)
        return OpStatus::InvalidValue;
    return fpgaPort->MemoryWrite(region.address, data, region.size);
}

OpStatus LimeSDR_XTRX::MemoryRead(std::shared_ptr<DataStorage> storage, Region region, void* data)
{
    if (storage == nullptr || storage->ownerDevice != this)
        return OpStatus::InvalidValue;
    return fpgaPort->MemoryRead(region.address, data, region.size);
}

OpStatus LimeSDR_XTRX::ClkTest(OEMTestReporter& reporter, TestData& results)
{
    OEMTestData test("PCIe Reference clock");
    reporter.OnStart(test);
    if (mFPGA->OEMTestSetup(FPGA::TestID::HostReferenceClock, 1.0) != OpStatus::Success)
    {
        reporter.OnFail(test, "timeout");
        return OpStatus::Error;
    }

    uint32_t addr[] = { 0x69, 0x69, 0x69 };
    uint32_t vals[3];
    try
    {
        fpgaPort->SPI(addr, vals, 3);
    } catch (...)
    {
        reporter.OnFail(test, "SPI failed");
        return OpStatus::IOFailure;
    }

    const bool pass = !(vals[0] == vals[1] && vals[1] == vals[2]);
    reporter.OnStepUpdate(
        test, "results: " + std::to_string(vals[0]) + "; " + std::to_string(vals[1]) + "; " + std::to_string(vals[2]));
    results.refClkPassed = pass;
    if (pass)
    {
        test.passed = true;
        reporter.OnSuccess(test);
        return OpStatus::Success;
    }
    else
    {
        reporter.OnFail(test, "values match");
        return OpStatus::Error;
    }
}

OpStatus LimeSDR_XTRX::GNSSTest(OEMTestReporter& reporter, TestData& results)
{
    OEMTestData test("GNSS");
    reporter.OnStart(test);

    FPGA::GatewareInfo gw = mFPGA->GetGatewareInfo();
    if (gw.version == 1) // original gateware
    {
        OpStatus status = mFPGA->OEMTestSetup(FPGA::TestID::GNSS, 1.0);
        results.gnssPassed = status == OpStatus::Success;
        if (status != OpStatus::Success)
        {
            reporter.OnFail(test, "timeout");
            return OpStatus::Error;
        }
    }
    else if (gw.version == 2) // litex based gateware
    {
        std::string path = mStreamPort->GetPathName();
        int flags = O_RDWR | O_NOCTTY | O_CLOEXEC;
        int fpos = path.find("/trx");
        path = path.substr(0, fpos);
        path.append("/uart0");

        int tty_fd = open(path.c_str(), flags);

        struct termios tty;

        if (tcgetattr(tty_fd, &tty) != 0)
        {
            printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        }

        tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
        tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
        tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
        tty.c_cflag |= CS8; // 8 bits per byte (most common)
        tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
        tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO; // Disable echo
        tty.c_lflag &= ~ECHOE; // Disable erasure
        tty.c_lflag &= ~ECHONL; // Disable new-line echo
        tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
        tty.c_iflag &=
            ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

        tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
        tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
        // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
        // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

        tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
        tty.c_cc[VMIN] = 0;

        // Set in/out baud rate to be 9600
        cfsetispeed(&tty, B9600);
        cfsetospeed(&tty, B9600);

        if (tcsetattr(tty_fd, TCSANOW, &tty) != 0)
        {
            printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        }

        const char outMessage[] = "$PMTK0*32\r\n";
        int written = write(tty_fd, outMessage, sizeof(outMessage));
        if (written != sizeof(outMessage))
        {
            reporter.OnFail(test, "Failed to write UART");
            return OpStatus::Error;
        }
        const std::string expectedAnswer = "$PMTK001,0,3*30";
        constexpr int bytesToRead = 256;
        char inMessage[bytesToRead + 16]; // + some padding
        memset(inMessage, 0, sizeof(inMessage));

        // UART is periodically sending messages, so the expected message might not be the first to be read
        for (int b = 0; b < bytesToRead;)
        {
            int bread = read(tty_fd, inMessage + b, sizeof(16));
            if (bread < 0)
            {
                close(tty_fd);
                reporter.OnFail(test, "Failed to read UART");
                return OpStatus::Error;
            }
            b += bread;
        }
        close(tty_fd);
        std::string searchStr(inMessage);
        size_t foundPos = searchStr.find(expectedAnswer);
        if (foundPos == std::string::npos)
        {
            reporter.OnFail(test, "Expected GNSS message not found");
            return OpStatus::Error;
        }
    }
    else
    {
        reporter.OnFail(test, "Unexpected gateware version.");
        return OpStatus::Error;
    }
    test.passed = true;
    reporter.OnSuccess(test);
    return OpStatus::Success;
}

class CustomParameterStash
{
  public:
    CustomParameterStash(SDRDevice* dev, const std::vector<CustomParameterIO>& args)
        : device(dev)
        , stash(args)
    {
        assert(dev);
        device->CustomParameterRead(stash);
    }
    ~CustomParameterStash() { device->CustomParameterWrite(stash); }

  private:
    SDRDevice* device;
    std::vector<CustomParameterIO> stash;
};

OpStatus LimeSDR_XTRX::VCTCXOTest(OEMTestReporter& reporter, TestData& results)
{
    OEMTestData test("VCTCXO");
    reporter.OnStart(test);

    unsigned count1;
    unsigned count2;

    std::vector<CustomParameterIO> params{ { limesdrxtrx::cp_vctcxo_dac.id, 0, "" } };

    try
    {
        OpStatus status;
        // Store current value, and restore it on return
        CustomParameterStash vctcxoStash(this, params);

        params[0].value = limesdrxtrx::cp_vctcxo_dac.minValue;
        status = CustomParameterWrite(params);
        if (status != OpStatus::Success)
            return status;

        status = mFPGA->OEMTestSetup(FPGA::TestID::VCTCXO, 1.0);
        if (status != OpStatus::Success)
        {
            reporter.OnFail(test, "timeout");
            return status;
        }

        uint32_t addr[] = { 0x72, 0x73 };
        uint32_t vals[2];
        if (mFPGA->ReadRegisters(addr, vals, 2) != OpStatus::Success)
        {
            reporter.OnFail(test, "IO failure");
            return OpStatus::IOFailure;
        }

        count1 = vals[0] + (vals[1] << 16);
        params[0].value = limesdrxtrx::cp_vctcxo_dac.maxValue;
        if (CustomParameterWrite(params) != OpStatus::Success)
        {
            reporter.OnFail(test, "IO failure");
            return OpStatus::IOFailure;
        }

        status = mFPGA->OEMTestSetup(FPGA::TestID::VCTCXO, 1.0);
        if (status != OpStatus::Success)
        {
            reporter.OnFail(test, "timeout");
            return status;
        }

        if (mFPGA->ReadRegisters(addr, vals, 2) != OpStatus::Success)
        {
            reporter.OnFail(test, "IO failure");
            return OpStatus::IOFailure;
        }

        count2 = vals[0] + (vals[1] << 16);
        std::string str = "Count : " + std::to_string(count1) + " (min); " + std::to_string(count2) + " (max)";
        results.vctcxoMinCount = count1;
        results.vctcxoMaxCount = count2;
        reporter.OnStepUpdate(test, str);

        const bool fail = (count1 + 20 > count2) || (count1 + 35 < count2);
        if (fail)
        {
            reporter.OnFail(test, "unexpected values");
            return OpStatus::Error;
        }
        results.vctcxoPassed = true;
        test.passed = true;
        reporter.OnSuccess(test);
    } catch (...)
    {
        reporter.OnFail(test, "IO failure");
        return OpStatus::IOFailure;
    }
    return OpStatus::Success;
}

OpStatus LimeSDR_XTRX::LMS7002_Test(OEMTestReporter& reporter, TestData& results)
{
    OEMTestData test("LMS7002M");
    reporter.OnStart(test);
    reporter.OnStepUpdate(test, "Registers test");

    auto& lmsControl = mLMSChips.at(0);

    try
    {
        lmsControl->SPI_write(0xA6, 0x0001);
        lmsControl->SPI_write(0x92, 0xFFFF);
        lmsControl->SPI_write(0x93, 0x03FF);
    } catch (...)
    {
        reporter.OnFail(test, "SPI failed");
        return OpStatus::IOFailure;
    }

    if (lmsControl->RegistersTest() != OpStatus::Success)
    {
        reporter.OnFail(test, "Registers test FAILED");
        return OpStatus::Error;
    }
    reporter.OnStepUpdate(test, "Registers test PASSED");

    LMS64CProtocol::DeviceReset(*mSerialPort, 0);

    reporter.OnStepUpdate(test, "External Reset line test");
    try
    {
        lmsControl->SPI_write(0x0020, 0xFFFD);
        OpStatus status;
        int val = lmsControl->SPI_read(0x20, true, &status);
        if (status != OpStatus::Success)
            return status;
        char str[64];
        std::snprintf(str, sizeof(str), "  Reg 0x20: Write value 0xFFFD, Read value 0x%04X", val);
        reporter.OnStepUpdate(test, str);
        if (val != 0xFFFD)
        {
            reporter.OnFail(test, "Register value mismatch");
            return OpStatus::Error;
        }

        LMS64CProtocol::DeviceReset(*mSerialPort, 0);
        val = lmsControl->SPI_read(0x20, true, &status);
        if (status != OpStatus::Success)
            return status;

        std::snprintf(str, sizeof(str), "  Reg 0x20: value after reset 0x0%4X", val);
        reporter.OnStepUpdate(test, str);
        if (val != 0xFFFF)
        {
            reporter.OnStepUpdate(test, "External Reset line test FAILED");
            return OpStatus::Error;
        }
    } catch (...)
    {
        reporter.OnFail(test, "SPI failed");
        return OpStatus::IOFailure;
    }
    results.lmsChipPassed = true;
    test.passed = true;
    reporter.OnSuccess(test);
    return OpStatus::Success;
}

OpStatus LimeSDR_XTRX::RunTestConfig(OEMTestReporter& reporter,
    TestData::RFData* results,
    const std::string& name,
    double LOFreq,
    int gain,
    int rxPath,
    double expectChA_dBFS,
    double expectChB_dBFS)
{
    SDRConfig config;
    config.channel[0].tx.sampleRate = config.channel[0].rx.sampleRate = 61.44e6;
    config.channel[0].rx.enabled = true;
    config.channel[0].tx.enabled = true;
    config.channel[0].tx.testSignal = ChannelConfig::Direction::TestSignal{ true, true }; // Test signal: DC
    config.channel[0].tx.testSignal.dcValue = complex16_t(0x7000, 0x7000);
    config.channel[0].tx.gain[eGainTypes::PAD] = 52;
    config.channel[0].tx.gain[eGainTypes::IAMP] = -18;

    const double tx_offset = 5e6;
    config.channel[0].rx.centerFrequency = LOFreq;
    config.channel[0].tx.centerFrequency = LOFreq + tx_offset;
    config.channel[0].rx.path = rxPath;
    config.channel[0].rx.gain[eGainTypes::GENERIC] = gain;

    // If RX H is chosen, use TX 1; else use TX 2
    config.channel[0].tx.path = rxPath == 1 ? 1 : 2;

    // same config for both channels
    config.channel[1] = config.channel[0];

    bool configPass = false;
    bool chAPass = false;
    bool chBPass = false;

    OpStatus status = Configure(config, 0);
    configPass = status == OpStatus::Success;

    RFTestInput args;
    args.rfTestTolerance_dB = 6;
    args.rfTestTolerance_Hz = 50e3;
    args.sampleRate = config.channel[0].rx.sampleRate;
    args.expectedPeakval_dBFS = expectChA_dBFS;
    args.expectedPeakFrequency = tx_offset;
    args.moduleIndex = 0;

    args.testName = name + " ChA";
    args.channelIndex = 0;

    RFTestOutput output{};
    if (configPass)
        chAPass = RunRFTest(*this, args, &reporter, &output) == OpStatus::Success;

    results[0].frequency = output.frequency;
    results[0].amplitude = output.amplitude_dBFS;
    results[0].passed = chAPass;

    args.testName = name + " ChB";
    args.channelIndex = 1;
    args.expectedPeakval_dBFS = expectChB_dBFS;
    if (configPass)
        chBPass = RunRFTest(*this, args, &reporter, &output) == OpStatus::Success;

    results[1].frequency = output.frequency;
    results[1].amplitude = output.amplitude_dBFS;
    results[1].passed = chBPass;

    bool pass = configPass && chAPass && chBPass;
    return pass ? OpStatus::Success : OpStatus::Error;
}

OpStatus LimeSDR_XTRX::RFTest(OEMTestReporter& reporter, TestData& results)
{
    OEMTestData test("RF");
    reporter.OnStart(test);
    //reporter.OnStepUpdate(test, "Note: The test should be run with loop connected between RF ports");
    reporter.OnStepUpdate(test, "->Configure LMS");

    if (Init() != OpStatus::Success)
    {
        test.passed = false;
        reporter.OnFail(test, "Failed to initialize device");
        return OpStatus::Error;
    }
    reporter.OnStepUpdate(test, "->Init Done");
    std::vector<OpStatus> statuses(3);

    statuses.push_back(RunTestConfig(reporter, results.lnal, "TX_2->LNA_L", 1000e6, 0, 2, -8, -8));
    statuses.push_back(RunTestConfig(reporter, results.lnaw, "TX_2->LNA_W", 2000e6, 14, 3, -8, -8));
    statuses.push_back(RunTestConfig(reporter, results.lnah, "TX_1->LNA_H", 3500e6, 35, 1, -8, -15));

    for (OpStatus s : statuses)
    {
        if (s != OpStatus::Success)
        {
            reporter.OnFail(test);
            return OpStatus::Error;
        }
    }
    test.passed = true;
    reporter.OnSuccess(test);
    return OpStatus::Success;
}

static std::string BoolToString(bool pass)
{
    return pass ? "PASS" : "FAIL";
}

LimeSDR_XTRX::TestData::TestData()
{
    memset(this, 0, sizeof(TestData));
}

OpStatus LimeSDR_XTRX::OEMTest(OEMTestReporter* reporter)
{
    TestData results;
    OEMTestData test("LimeSDR-XTRX OEM Test");
    reporter->OnStart(test);
    bool pass = true;
    pass &= ClkTest(*reporter, results) == OpStatus::Success;
    pass &= VCTCXOTest(*reporter, results) == OpStatus::Success;
    pass &= GNSSTest(*reporter, results) == OpStatus::Success;
    pass &= LMS7002_Test(*reporter, results) == OpStatus::Success;
    const bool rfPassed = RFTest(*reporter, results) == OpStatus::Success;
    pass &= rfPassed;

    reporter->ReportColumn("PCIe Ref Clk", BoolToString(results.refClkPassed));
    reporter->ReportColumn("VCTCXO", BoolToString(results.vctcxoPassed));
    reporter->ReportColumn("VCTCXO min", std::to_string(results.vctcxoMinCount));
    reporter->ReportColumn("VCTCXO max", std::to_string(results.vctcxoMaxCount));
    reporter->ReportColumn("GNSS", BoolToString(results.gnssPassed));
    reporter->ReportColumn("LMS7002M", BoolToString(results.lmsChipPassed));
    reporter->ReportColumn("RF", BoolToString(rfPassed));
    reporter->ReportColumn("TX_2->LNA_L A", std::to_string(results.lnal[0].amplitude));
    reporter->ReportColumn("TX_2->LNA_L B", std::to_string(results.lnal[1].amplitude));
    reporter->ReportColumn("TX_2->LNA_W A", std::to_string(results.lnaw[0].amplitude));
    reporter->ReportColumn("TX_2->LNA_W B", std::to_string(results.lnaw[1].amplitude));
    reporter->ReportColumn("TX_1->LNA_H A", std::to_string(results.lnah[0].amplitude));
    reporter->ReportColumn("TX_1->LNA_H B", std::to_string(results.lnah[1].amplitude));

    if (pass)
    {
        reporter->OnSuccess(test);
        return OpStatus::Success;
    }
    else
    {
        reporter->OnFail(test);
        return OpStatus::Error;
    }
}

OpStatus LimeSDR_XTRX::WriteSerialNumber(uint64_t serialNumber)
{
    std::vector<uint8_t> bytes(sizeof(serialNumber));
    for (size_t i = 0; i < sizeof(serialNumber); ++i)
        bytes[i] = serialNumber >> (8 * i);
    OpStatus status = LMS64CProtocol::WriteSerialNumber(*mSerialPort, bytes);

    if (status == OpStatus::Success)
        mDeviceDescriptor.serialNumber = serialNumber;
    return status;
}

OpStatus LimeSDR_XTRX::SetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t path)
{
    OpStatus status = LMS7002M_SDRDevice::SetAntenna(moduleIndex, trx, channel, path);
    if (status != OpStatus::Success)
        return status;
    LMSSetPath(trx, channel, path);
    return OpStatus::Success;
}

std::unique_ptr<lime::RFStream> LimeSDR_XTRX::StreamCreate(const StreamConfig& config, uint8_t moduleIndex)
{
    if (mStreamPort.get() == nullptr)
    {
        lime::warning("XTRX RF data stream is not available");
        return std::unique_ptr<RFStream>(nullptr);
    }

    std::shared_ptr<LimePCIe> trxPort{ mStreamPort };
    auto rxdma = std::make_shared<LimePCIeDMA>(trxPort, DataTransferDirection::DeviceToHost);
    auto txdma = std::make_shared<LimePCIeDMA>(trxPort, DataTransferDirection::HostToDevice);

    std::unique_ptr<TRXLooper> streamer = std::make_unique<TRXLooper>(rxdma, txdma, mFPGA.get(), mLMSChips.at(0).get(), 0);
    if (!streamer)
        return streamer;

    if (mCallback_logMessage)
        streamer->SetMessageLogCallback(mCallback_logMessage);
    OpStatus status = streamer->Setup(config);
    if (status != OpStatus::Success)
        return std::unique_ptr<RFStream>(nullptr);
    return streamer;
}

} //namespace lime
