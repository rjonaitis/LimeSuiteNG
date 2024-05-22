/**
@file LMS7002M.cpp
@author Lime Microsystems (www.limemicro.com)
@brief Implementation of LMS7002M transceiver configuring
*/

#define _USE_MATH_DEFINES
#include "limesuiteng/LMS7002M.h"
#include "limesuiteng/embedded/lms7002m/lms7002m.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <ciso646>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string_view>
#include <unordered_set>
#include <thread>

#ifdef __GNUC__
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wold-style-cast"
#endif
#include "cpp-feather-ini-parser/INI.h"
#ifdef __GNUC__
    #pragma GCC diagnostic pop
#endif

#include "DSP/GFIR/lms_gfir.h"
#include "limesuiteng/types.h"
#include "comms/IComms.h"
#include "LMS7002M_RegistersMap.h"
#include "LMS7002MCSR_Data.h"
#include "limesuiteng/LMS7002MCSR.h"
#include "limesuiteng/Logger.h"
#include "mcu_programs.h"
#include "MCU_BD.h"
#include "utilities/toString.h"

using namespace lime;
using namespace LMS7002MCSR_Data;
using namespace std::literals::string_literals;
using namespace std::literals::string_view_literals;

constexpr std::array<std::array<float_type, 2>, 3> LMS7002M::gVCO_frequency_table{
    { { 3800e6, 5222e6 }, { 4961e6, 6754e6 }, { 6306e6, 7714e6 } }
};

constexpr LMS7002M::Channel IntToChannel(int channel)
{
    return channel > 0 ? LMS7002M::Channel::ChB : LMS7002M::Channel::ChA;
}

// Module addresses needs to be sorted in ascending order
const std::vector<LMS7002M::ReadOnlyRegister> LMS7002M::readOnlyRegisters{
    { 0x002F, 0x0000 },
    { 0x008C, 0x0FFF },
    { 0x00A8, 0x007F },
    { 0x00A9, 0x0000 },
    { 0x00AA, 0x0000 },
    { 0x00AB, 0x0000 },
    { 0x00AC, 0x0000 },
    { 0x0123, 0x003F },
    { 0x0209, 0x0000 },
    { 0x020A, 0x0000 },
    { 0x020B, 0x0000 },
    { 0x040E, 0x0000 },
    { 0x040F, 0x0000 },
};

const std::map<LMS7002M::MemorySection, std::array<uint16_t, 2>> LMS7002M::MemorySectionAddresses{
    { LMS7002M::MemorySection::LimeLight, { 0x0020, 0x002F } },
    { LMS7002M::MemorySection::EN_DIR, { 0x0081, 0x0081 } },
    { LMS7002M::MemorySection::AFE, { 0x0082, 0x0082 } },
    { LMS7002M::MemorySection::BIAS, { 0x0084, 0x0084 } },
    { LMS7002M::MemorySection::XBUF, { 0x0085, 0x0085 } },
    { LMS7002M::MemorySection::CGEN, { 0x0086, 0x008C } },
    { LMS7002M::MemorySection::LDO, { 0x0092, 0x00A7 } },
    { LMS7002M::MemorySection::BIST, { 0x00A8, 0x00AC } },
    { LMS7002M::MemorySection::CDS, { 0x00AD, 0x00AE } },
    { LMS7002M::MemorySection::TRF, { 0x0100, 0x0104 } },
    { LMS7002M::MemorySection::TBB, { 0x0105, 0x010B } },
    { LMS7002M::MemorySection::RFE, { 0x010C, 0x0114 } },
    { LMS7002M::MemorySection::RBB, { 0x0115, 0x011A } },
    { LMS7002M::MemorySection::SX, { 0x011C, 0x0124 } },
    { LMS7002M::MemorySection::TRX_GAIN, { 0x0125, 0x0126 } },
    { LMS7002M::MemorySection::TxTSP, { 0x0200, 0x020C } },
    { LMS7002M::MemorySection::TxNCO, { 0x0240, 0x0261 } },
    { LMS7002M::MemorySection::TxGFIR1, { 0x0280, 0x02A7 } },
    { LMS7002M::MemorySection::TxGFIR2, { 0x02C0, 0x02E7 } },
    { LMS7002M::MemorySection::TxGFIR3a, { 0x0300, 0x0327 } },
    { LMS7002M::MemorySection::TxGFIR3b, { 0x0340, 0x0367 } },
    { LMS7002M::MemorySection::TxGFIR3c, { 0x0380, 0x03A7 } },
    { LMS7002M::MemorySection::RxTSP, { 0x0400, 0x040F } },
    { LMS7002M::MemorySection::RxNCO, { 0x0440, 0x0461 } },
    { LMS7002M::MemorySection::RxGFIR1, { 0x0480, 0x04A7 } },
    { LMS7002M::MemorySection::RxGFIR2, { 0x04C0, 0x04E7 } },
    { LMS7002M::MemorySection::RxGFIR3a, { 0x0500, 0x0527 } },
    { LMS7002M::MemorySection::RxGFIR3b, { 0x0540, 0x0567 } },
    { LMS7002M::MemorySection::RxGFIR3c, { 0x0580, 0x05A7 } },
    { LMS7002M::MemorySection::RSSI_DC_CALIBRATION, { 0x05C0, 0x05CC } },
    { LMS7002M::MemorySection::RSSI_PDET_TEMP_CONFIG, { 0x0600, 0x0606 } },
    { LMS7002M::MemorySection::RSSI_DC_CONFIG, { 0x0640, 0x0641 } },
};

static_assert(OpStatus::Success == static_cast<lime::OpStatus>(lime_Result_Success));
static_assert(OpStatus::Error == static_cast<lime::OpStatus>(lime_Result_Error));
static_assert(OpStatus::NotImplemented == static_cast<lime::OpStatus>(lime_Result_NotImplemented));
static_assert(OpStatus::IOFailure == static_cast<lime::OpStatus>(lime_Result_IOFailure));
static_assert(OpStatus::InvalidValue == static_cast<lime::OpStatus>(lime_Result_InvalidValue));
static_assert(OpStatus::FileNotFound == static_cast<lime::OpStatus>(lime_Result_FileNotFound));
static_assert(OpStatus::OutOfRange == static_cast<lime::OpStatus>(lime_Result_OutOfRange));
static_assert(OpStatus::NotSupported == static_cast<lime::OpStatus>(lime_Result_NotSupported));
static_assert(OpStatus::Timeout == static_cast<lime::OpStatus>(lime_Result_Timeout));
static_assert(OpStatus::Busy == static_cast<lime::OpStatus>(lime_Result_Busy));
static_assert(OpStatus::Aborted == static_cast<lime::OpStatus>(lime_Result_Aborted));
static_assert(OpStatus::PermissionDenied == static_cast<lime::OpStatus>(lime_Result_PermissionDenied));
static_assert(OpStatus::NotConnected == static_cast<lime::OpStatus>(lime_Result_NotConnected));

static OpStatus ResultToStatus(lime_Result result)
{
    return static_cast<lime::OpStatus>(result);
}

static_assert(LMS7002M::ClockID::CLK_REFERENCE == static_cast<LMS7002M::ClockID>(LMS7002M_CLK_REFERENCE));
static_assert(LMS7002M::ClockID::CLK_SXR == static_cast<LMS7002M::ClockID>(LMS7002M_CLK_SXR));
static_assert(LMS7002M::ClockID::CLK_SXT == static_cast<LMS7002M::ClockID>(LMS7002M_CLK_SXT));
static_assert(LMS7002M::ClockID::CLK_CGEN == static_cast<LMS7002M::ClockID>(LMS7002M_CLK_CGEN));
static_assert(LMS7002M::ClockID::CLK_RXTSP == static_cast<LMS7002M::ClockID>(LMS7002M_CLK_RXTSP));
static_assert(LMS7002M::ClockID::CLK_TXTSP == static_cast<LMS7002M::ClockID>(LMS7002M_CLK_TXTSP));

static_assert(LMS7002M::Channel::ChA == static_cast<LMS7002M::Channel>(LMS7002M_CHANNEL_A));
static_assert(LMS7002M::Channel::ChB == static_cast<LMS7002M::Channel>(LMS7002M_CHANNEL_B));
static_assert(LMS7002M::Channel::ChAB == static_cast<LMS7002M::Channel>(LMS7002M_CHANNEL_AB));
static_assert(LMS7002M::Channel::ChSXR == static_cast<LMS7002M::Channel>(LMS7002M_CHANNEL_SXR));
static_assert(LMS7002M::Channel::ChSXT == static_cast<LMS7002M::Channel>(LMS7002M_CHANNEL_SXT));

static_assert(LMS7002M::PathRFE::NONE == static_cast<LMS7002M::PathRFE>(LMS7002M_PATH_RFE_NONE));
static_assert(LMS7002M::PathRFE::LNAH == static_cast<LMS7002M::PathRFE>(LMS7002M_PATH_RFE_LNAH));
static_assert(LMS7002M::PathRFE::LNAL == static_cast<LMS7002M::PathRFE>(LMS7002M_PATH_RFE_LNAL));
static_assert(LMS7002M::PathRFE::LNAW == static_cast<LMS7002M::PathRFE>(LMS7002M_PATH_RFE_LNAW));
static_assert(LMS7002M::PathRFE::LB1 == static_cast<LMS7002M::PathRFE>(LMS7002M_PATH_RFE_LB1));
static_assert(LMS7002M::PathRFE::LB2 == static_cast<LMS7002M::PathRFE>(LMS7002M_PATH_RFE_LB2));

static_assert(LMS7002M::VCO_Module::VCO_CGEN == static_cast<LMS7002M::VCO_Module>(LMS7002M_VCO_CGEN));
static_assert(LMS7002M::VCO_Module::VCO_SXR == static_cast<LMS7002M::VCO_Module>(LMS7002M_VCO_SXR));
static_assert(LMS7002M::VCO_Module::VCO_SXT == static_cast<LMS7002M::VCO_Module>(LMS7002M_VCO_SXT));

/** @brief Switches LMS7002M SPI to requested channel and restores previous channel when going out of scope */
class ChannelScope
{
  public:
    /**
   * @brief Saves the current channel and restores it at scope exit.
   * @param chip The chip to use.
   * @param useCache Whether to use caching or not.
   */
    ChannelScope(LMS7002M* chip, bool useCache = false)
        : mChip(chip)
        , mStoredValue(chip->GetActiveChannel(!useCache))
        , mNeedsRestore(true)
    {
    }

    /**
      @brief Convenient constructor when using explicit MAC value.
      @param chip The chip to use.
      @param mac The channel to use.
      @param useCache Whether to use caching or not.
     */
    ChannelScope(LMS7002M* chip, LMS7002M::Channel mac, bool useCache = false)
        : mChip(chip)
        , mStoredValue(chip->GetActiveChannel(!useCache))
        , mNeedsRestore(false)
    {
        if (mStoredValue == mac)
            return;

        mChip->SetActiveChannel(mac);
        mNeedsRestore = true;
    }

    /**
      @brief Convenient constructor when using channel index starting from 0.
      @param chip The chip to use.
      @param index The channel index.
      @param useCache Whether to use caching or not.
     */
    ChannelScope(LMS7002M* chip, uint8_t index, bool useCache = false)
        : mChip(chip)
        , mNeedsRestore(false)
    {
        assert(index < 2);
        mStoredValue = chip->GetActiveChannel(!useCache);
        auto expectedChannel = IntToChannel(index);
        if (mStoredValue == expectedChannel)
            return;

        mChip->SetActiveChannel(expectedChannel);
        mNeedsRestore = true;
    }

    /** @brief Destroy the Channel Scope object and reset the active channel. */
    ~ChannelScope()
    {
        if (mNeedsRestore)
            mChip->SetActiveChannel(mStoredValue);
    }

  private:
    LMS7002M* mChip; ///< The chip to modify
    LMS7002M::Channel mStoredValue; ///< The channel to restore to
    bool mNeedsRestore; ///< Whether the channel needs restoring or not
};

/** @brief Sets connection which is used for data communication with chip
*/
void LMS7002M::SetConnection(std::shared_ptr<ISPI> port)
{
    controlPort = port;

    if (controlPort == nullptr)
    {
        return;
    }

    unsigned byte_array_size = 0;
    unsigned chipRev = this->Get_SPI_Reg_bits(MASK, true);
    if (chipRev >= 1)
        byte_array_size = 1024 * 16;
    else
        byte_array_size = 1024 * 8;
    mcuControl->Initialize(port, byte_array_size);
}

static int spi16_transact(const uint32_t* mosi, uint32_t* miso, uint32_t count, void* userData)
{
    LMS7002M* chip = reinterpret_cast<LMS7002M*>(userData);
    for (uint32_t i = 0; i < count; ++i)
    {
        if (mosi[i] & (1 << 31))
        {
            uint16_t addr = mosi[i] >> 16;
            addr &= 0x7FFF; // clear write bit for now
            uint16_t value = mosi[i] & 0xFFFF;
            chip->SPI_write(addr, value);
        }
        else
        {
            uint16_t addr = mosi[i] >> 16;
            uint16_t value = 0;
            value = chip->SPI_read(addr);
            if (miso)
                miso[i] = value;
        }
    }
    return 0;
}

static void log_hook(int logLevel, const char* message, void* userData)
{
    LogLevel level = static_cast<lime::LogLevel>(logLevel);
    log(level, std::string{ message });
}

/** @brief Creates LMS7002M main control object.
It requires IConnection to be set by SetConnection() to communicate with chip
*/
LMS7002M::LMS7002M(std::shared_ptr<ISPI> port)
    : mCallback_onCGENChange(nullptr)
    , mCallback_onCGENChange_userData(nullptr)
    , useCache(0)
    , mRegistersMap(new LMS7002M_RegistersMap())
    , controlPort(port)
    , mC_impl(nullptr)
{
    struct lms7002m_hooks hooks;
    memset(&hooks, 0, sizeof(hooks));

    hooks.spi16_userData = this;
    hooks.spi16_transact = spi16_transact;
    hooks.log = log_hook;
    hooks.log_userData = this;
    hooks.on_cgen_frequency_changed_userData = this;
    hooks.on_cgen_frequency_changed = [](void* userData) -> void {
        LMS7002M* chip = reinterpret_cast<LMS7002M*>(userData);
        if (chip->mCallback_onCGENChange)
            chip->mCallback_onCGENChange(chip->mCallback_onCGENChange_userData);
    };

    mC_impl = lms7002m_create(&hooks);
    if (mC_impl == nullptr)
        lime::error("Failed to initialize LMS7002M C implementation");
    lms7002m_set_reference_clock(mC_impl, 30.72e6);

    opt_gain_tbb[0] = -1;
    opt_gain_tbb[1] = -1;

    std::vector<const LMS7002MCSR_Data::CSRegister*> parameterList;
    parameterList.reserve(static_cast<int>(LMS7002MCSR::ENUM_COUNT));
    for (int i = 0; i < static_cast<int>(LMS7002MCSR::ENUM_COUNT); ++i)
    {
        const LMS7002MCSR_Data::CSRegister& parameter = GetRegister(static_cast<LMS7002MCSR>(i));
        parameterList.push_back(&parameter);
    }
    mRegistersMap->InitializeDefaultValues(parameterList);
    mcuControl = new MCU_BD();
    mcuControl->Initialize(controlPort);
}

LMS7002M::~LMS7002M()
{
    lms7002m_destroy(mC_impl);
    delete mcuControl;
    delete mRegistersMap;
}

OpStatus LMS7002M::SetActiveChannel(const Channel ch)
{
    lime_Result result = lms7002m_set_active_channel(mC_impl, static_cast<lms7002m_channel>(ch));
    return ResultToStatus(result);
}

LMS7002M::Channel LMS7002M::GetActiveChannel(bool fromChip)
{
    auto result{ GetActiveChannelIndex(fromChip) };
    return static_cast<Channel>(result);
}

size_t LMS7002M::GetActiveChannelIndex(bool fromChip)
{
    uint8_t result = lms7002m_get_active_channel(mC_impl);
    return result;
}

OpStatus LMS7002M::EnableChannel(TRXDir dir, const uint8_t channel, const bool enable)
{
    lime_Result result = lms7002m_enable_channel(mC_impl, dir == TRXDir::Tx, static_cast<lms7002m_channel>(channel), enable);
    return ResultToStatus(result);
}

OpStatus LMS7002M::ResetChip()
{
    OpStatus status;

    const std::vector<uint16_t> usedAddresses = mRegistersMap->GetUsedAddresses(0);

    std::vector<uint16_t> addrs;
    addrs.reserve(usedAddresses.size() + 2);
    std::vector<uint16_t> values;
    values.reserve(usedAddresses.size() + 2);

    addrs.push_back(0x0006); // SPISW_CTRL
    values.push_back(0x0000); // ensure baseband is controlling SPI

    addrs.push_back(0x0020);
    uint16_t x0020default = mRegistersMap->GetDefaultValue(0x0020);
    values.push_back(x0020default | 0x3); // enable simultaneous A&B write

    for (uint16_t addr : usedAddresses)
    {
        if (addr == 0x0020) // skip address containing MAC, to continue writing both channels
            continue;
        addrs.push_back(addr);
        values.push_back(mRegistersMap->GetDefaultValue(addr));
    }
    addrs.push_back(0x0020);
    values.push_back((x0020default & ~0x3) | 0x1); // back to A channel only

    status = SPI_write_batch(addrs.data(), values.data(), addrs.size(), true);
    status = Modify_SPI_Reg_bits(LMS7002MCSR::MIMO_SISO, 0); //enable B channel after reset
    return status;
}

OpStatus LMS7002M::SoftReset()
{
    lime_Result result = lms7002m_soft_reset(mC_impl);
    return ResultToStatus(result);
}

OpStatus LMS7002M::ResetLogicRegisters()
{
    lime_Result result = lms7002m_reset_logic_registers(mC_impl);
    return ResultToStatus(result);
}

OpStatus LMS7002M::LoadConfigLegacyFile(const std::string& filename)
{
    std::ifstream f(filename);
    if (f.good() == false) //file not found
    {
        f.close();
        return ReportError(OpStatus::FileNotFound, "LoadConfigLegacyFile(%s) - file not found", filename.c_str());
    }
    f.close();

    uint16_t addr = 0;
    uint16_t value = 0;
    OpStatus status;
    typedef INI<std::string, std::string, std::string> ini_t;
    ini_t parser(filename, true);
    if (parser.select("FILE INFO"s) == false)
        return ReportError(
            OpStatus::InvalidValue, "LoadConfigLegacyFile(%s) - invalid format, missing FILE INFO section", filename.c_str());

    std::string type{};
    type = parser.get("type"s, "undefined"s);

    if (type.find("LMS7002 configuration"sv) == std::string::npos)
    {
        return ReportError(
            OpStatus::InvalidValue, "LoadConfigLegacyFile(%s) - invalid format, missing LMS7002 configuration", filename.c_str());
    }

    int fileVersion = 0;
    fileVersion = parser.get("version"sv, 0);

    std::vector<uint16_t> addrToWrite;
    std::vector<uint16_t> dataToWrite;
    if (fileVersion != 1)
    {
        return ReportError(OpStatus::InvalidValue, "LoadConfigLegacyFile(%s) - invalid format", filename.c_str());
    }

    ChannelScope scope(this);

    if (parser.select("Reference clocks"s))
    {
        this->SetReferenceClk_SX(TRXDir::Rx, parser.get("SXR reference frequency MHz"sv, 30.72) * 1e6);
        this->SetReferenceClk_SX(TRXDir::Tx, parser.get("SXT reference frequency MHz"sv, 30.72) * 1e6);
    }

    if (parser.select("LMS7002 registers ch.A"s) == true)
    {
        ini_t::sectionsit_t section = parser.sections.find("LMS7002 registers ch.A"s);

        uint16_t x0020_value = 0;
        this->SetActiveChannel(Channel::ChA); //select A channel
        for (ini_t::keysit_t pairs = section->second->begin(); pairs != section->second->end(); pairs++)
        {
            sscanf(pairs->first.c_str(), "%hx", &addr);
            sscanf(pairs->second.c_str(), "%hx", &value);
            if (addr == MAC.address) //skip register containing channel selection
            {
                x0020_value = value;
                continue;
            }
            addrToWrite.push_back(addr);
            dataToWrite.push_back(value);
        }
        status = SPI_write_batch(&addrToWrite[0], &dataToWrite[0], addrToWrite.size(), true);
        if (status != OpStatus::Success && controlPort != nullptr)
            return status;

        //parse FCW or PHO
        if (parser.select("NCO Rx ch.A"s) == true)
        {
            char varname[64];
            int mode = Get_SPI_Reg_bits(LMS7002MCSR::MODE_RX);
            if (mode == 0) //FCW
            {
                for (int i = 0; i < 16; ++i)
                {
                    std::snprintf(varname, sizeof(varname), "FCW%02i", i);
                    SetNCOFrequency(TRXDir::Rx, i, parser.get(varname, 0.0));
                }
            }
            else
            {
                for (int i = 0; i < 16; ++i)
                {
                    std::snprintf(varname, sizeof(varname), "PHO%02i", i);
                    SetNCOPhaseOffset(TRXDir::Rx, i, parser.get(varname, 0.0));
                }
            }
        }
        if (parser.select("NCO Tx ch.A"s) == true)
        {
            char varname[64];
            int mode = Get_SPI_Reg_bits(LMS7002MCSR::MODE_TX);
            if (mode == 0) //FCW
            {
                for (int i = 0; i < 16; ++i)
                {
                    std::snprintf(varname, sizeof(varname), "FCW%02i", i);
                    SetNCOFrequency(TRXDir::Tx, i, parser.get(varname, 0.0));
                }
            }
            else
            {
                for (int i = 0; i < 16; ++i)
                {
                    std::snprintf(varname, sizeof(varname), "PHO%02i", i);
                    SetNCOPhaseOffset(TRXDir::Tx, i, parser.get(varname, 0.0));
                }
            }
        }
        status = SPI_write(0x0020, x0020_value);
        if (status != OpStatus::Success && controlPort != nullptr)
            return status;
    }

    this->SetActiveChannel(Channel::ChB);

    if (parser.select("LMS7002 registers ch.B"s) == true)
    {
        addrToWrite.clear();
        dataToWrite.clear();
        ini_t::sectionsit_t section = parser.sections.find("LMS7002 registers ch.B"s);
        for (ini_t::keysit_t pairs = section->second->begin(); pairs != section->second->end(); pairs++)
        {
            sscanf(pairs->first.c_str(), "%hx", &addr);
            sscanf(pairs->second.c_str(), "%hx", &value);
            addrToWrite.push_back(addr);
            dataToWrite.push_back(value);
        }
        this->SetActiveChannel(Channel::ChB); //select B channel
        status = SPI_write_batch(&addrToWrite[0], &dataToWrite[0], addrToWrite.size(), true);
        if (status != OpStatus::Success && controlPort != nullptr)
            return status;

        //parse FCW or PHO
        if (parser.select("NCO Rx ch.B"s) == true)
        {
            char varname[64];
            int mode = Get_SPI_Reg_bits(LMS7002MCSR::MODE_RX);
            if (mode == 0) //FCW
            {
                for (int i = 0; i < 16; ++i)
                {
                    std::snprintf(varname, sizeof(varname), "FCW%02i", i);
                    SetNCOFrequency(TRXDir::Rx, i, parser.get(varname, 0.0));
                }
            }
            else
            {
                for (int i = 0; i < 16; ++i)
                {
                    std::snprintf(varname, sizeof(varname), "PHO%02i", i);
                    SetNCOPhaseOffset(TRXDir::Rx, i, parser.get(varname, 0.0));
                }
            }
        }
        if (parser.select("NCO Tx ch.A"s) == true)
        {
            char varname[64];
            int mode = Get_SPI_Reg_bits(LMS7002MCSR::MODE_TX);
            if (mode == 0) //FCW
            {
                for (int i = 0; i < 16; ++i)
                {
                    std::snprintf(varname, sizeof(varname), "FCW%02i", i);
                    SetNCOFrequency(TRXDir::Tx, i, parser.get(varname, 0.0));
                }
            }
            else
            {
                for (int i = 0; i < 16; ++i)
                {
                    std::snprintf(varname, sizeof(varname), "PHO%02i", i);
                    SetNCOPhaseOffset(TRXDir::Tx, i, parser.get(varname, 0.0));
                }
            }
        }
    }
    return OpStatus::Success;
}

OpStatus LMS7002M::LoadConfig(const std::string& filename, bool tuneDynamicValues)
{
    std::ifstream f(filename);
    if (f.good() == false) //file not found
    {
        f.close();
        return ReportError(OpStatus::FileNotFound, "LoadConfig(%s) - file not found", filename.c_str());
    }
    f.close();

    uint16_t addr = 0;
    uint16_t value = 0;

    OpStatus status;
    typedef INI<std::string, std::string, std::string> ini_t;
    ini_t parser(filename, true);
    if (parser.select("file_info"s) == false)
    {
        //try loading as legacy format
        status = LoadConfigLegacyFile(filename);
        this->SetActiveChannel(Channel::ChA);
        return status;
    }
    std::string type{};
    type = parser.get("type"s, "undefined"s);

    if (type.find("lms7002m_minimal_config"sv) == std::string::npos)
    {
        return ReportError(
            OpStatus::InvalidValue, "LoadConfig(%s) - invalid format, missing lms7002m_minimal_config", filename.c_str());
    }

    int fileVersion = 0;
    fileVersion = parser.get("version"sv, 0);

    std::vector<uint16_t> addrToWrite;
    std::vector<uint16_t> dataToWrite;

    if (fileVersion == 1)
    {
        ChannelScope scope(this);
        if (parser.select("lms7002_registers_a"s) == true)
        {
            ini_t::sectionsit_t section = parser.sections.find("lms7002_registers_a"s);

            uint16_t x0020_value = 0;
            this->SetActiveChannel(Channel::ChA); //select A channel
            for (ini_t::keysit_t pairs = section->second->begin(); pairs != section->second->end(); pairs++)
            {
                sscanf(pairs->first.c_str(), "%hx", &addr);
                sscanf(pairs->second.c_str(), "%hx", &value);
                if (addr == MAC.address) //skip register containing channel selection
                {
                    x0020_value = value;
                    continue;
                }

                if (addr >= 0x5C3 && addr <= 0x5CA) //enable analog DC correction
                {
                    addrToWrite.push_back(addr);
                    dataToWrite.push_back(value & 0x3FFF);
                    addrToWrite.push_back(addr);
                    dataToWrite.push_back(value | 0x8000);
                }
                else
                {
                    addrToWrite.push_back(addr);
                    dataToWrite.push_back(value);
                }
            }

            status = SPI_write_batch(&addrToWrite[0], &dataToWrite[0], addrToWrite.size(), true);
            if (status != OpStatus::Success && controlPort != nullptr)
                return status;
            status = SPI_write(0x0020, x0020_value);
            if (status != OpStatus::Success && controlPort != nullptr)
                return status;
            this->SetActiveChannel(Channel::ChB);
            if (status != OpStatus::Success && controlPort != nullptr)
                return status;
        }

        if (parser.select("lms7002_registers_b"s) == true)
        {
            addrToWrite.clear();
            dataToWrite.clear();
            ini_t::sectionsit_t section = parser.sections.find("lms7002_registers_b"s);
            for (ini_t::keysit_t pairs = section->second->begin(); pairs != section->second->end(); pairs++)
            {
                sscanf(pairs->first.c_str(), "%hx", &addr);
                sscanf(pairs->second.c_str(), "%hx", &value);
                addrToWrite.push_back(addr);
                dataToWrite.push_back(value);
            }
            this->SetActiveChannel(Channel::ChB); //select B channel
            status = SPI_write_batch(&addrToWrite[0], &dataToWrite[0], addrToWrite.size(), true);
            if (status != OpStatus::Success && controlPort != nullptr)
                return status;
        }

        parser.select("reference_clocks"s);
        this->SetReferenceClk_SX(TRXDir::Rx, parser.get("sxr_ref_clk_mhz"sv, 30.72) * 1e6);
        this->SetReferenceClk_SX(TRXDir::Tx, parser.get("sxt_ref_clk_mhz"sv, 30.72) * 1e6);
    }

    ResetLogicRegisters();

    if (tuneDynamicValues)
    {
        Modify_SPI_Reg_bits(LMS7002MCSR::MAC, 2);
        if (!Get_SPI_Reg_bits(LMS7002MCSR::PD_VCO))
            TuneVCO(VCO_Module::VCO_SXT);
        Modify_SPI_Reg_bits(LMS7002MCSR::MAC, 1);
        if (!Get_SPI_Reg_bits(LMS7002MCSR::PD_VCO))
            TuneVCO(VCO_Module::VCO_SXR);
        if (!Get_SPI_Reg_bits(LMS7002MCSR::PD_VCO_CGEN))
        {
            TuneVCO(VCO_Module::VCO_CGEN);
            if (mCallback_onCGENChange)
                return mCallback_onCGENChange(mCallback_onCGENChange_userData);
        }
    }
    this->SetActiveChannel(Channel::ChA);
    return OpStatus::Success;
}

OpStatus LMS7002M::SaveConfig(const std::string& filename)
{
    std::ofstream fout;
    fout.open(filename);
    fout << "[file_info]"sv << std::endl;
    fout << "type=lms7002m_minimal_config"sv << std::endl;
    fout << "version=1"sv << std::endl;

    char addr[80];
    char value[80];

    ChannelScope scope(this);

    std::vector<uint16_t> addrToRead;
    for (const auto& memorySectionPair : MemorySectionAddresses)
        for (uint16_t addr = memorySectionPair.second[0]; addr <= memorySectionPair.second[1]; ++addr)
            addrToRead.push_back(addr);

    std::vector<uint16_t> dataReceived;
    dataReceived.resize(addrToRead.size(), 0);

    fout << "[lms7002_registers_a]"sv << std::endl;
    this->SetActiveChannel(Channel::ChA);
    for (uint16_t i = 0; i < addrToRead.size(); ++i)
    {
        if (addrToRead[i] >= 0x5C3 && addrToRead[i] <= 0x5CA)
            SPI_write(addrToRead[i], 0x4000); //perform read-back from DAC
        dataReceived[i] = Get_SPI_Reg_bits(addrToRead[i], 15, 0, false);

        //registers 0x5C3 - 0x53A return inverted value field when DAC value read-back is performed
        if (addrToRead[i] >= 0x5C3 && addrToRead[i] <= 0x5C6 && (dataReceived[i] & 0x400)) //sign bit 10
            dataReceived[i] = 0x400 | (~dataReceived[i] & 0x3FF); //magnitude bits  9:0
        else if (addrToRead[i] >= 0x5C7 && addrToRead[i] <= 0x5CA && (dataReceived[i] & 0x40)) //sign bit 6
            dataReceived[i] = 0x40 | (~dataReceived[i] & 0x3F); //magnitude bits  5:0
        else if (addrToRead[i] == 0x5C2)
            dataReceived[i] &= 0xFF00; //do not save calibration start triggers
        std::snprintf(addr, sizeof(addr), "0x%04X", addrToRead[i]);
        std::snprintf(value, sizeof(value), "0x%04X", dataReceived[i]);
        fout << addr << "="sv << value << std::endl;
        // add parameter name/value as comments
        for (int p = 0; p < static_cast<int>(LMS7002MCSR::ENUM_COUNT); ++p)
        {
            const LMS7002MCSR_Data::CSRegister& parameter = GetRegister(static_cast<LMS7002MCSR>(p));
            if (parameter.address == addrToRead[i])
                fout << "//"sv << parameter.name << " : "sv << Get_SPI_Reg_bits(static_cast<LMS7002MCSR>(p)) << std::endl;
        }
    }

    fout << "[lms7002_registers_b]"sv << std::endl;
    addrToRead.clear(); //add only B channel addresses
    for (const auto& memorySectionPair : MemorySectionAddresses)
        if (memorySectionPair.first != MemorySection::RSSI_DC_CALIBRATION)
            for (uint16_t addr = memorySectionPair.second[0]; addr <= memorySectionPair.second[1]; ++addr)
                if (addr >= 0x0100)
                    addrToRead.push_back(addr);

    this->SetActiveChannel(Channel::ChB);
    for (uint16_t i = 0; i < addrToRead.size(); ++i)
    {
        dataReceived[i] = Get_SPI_Reg_bits(addrToRead[i], 15, 0, false);
        std::snprintf(addr, sizeof(addr), "0x%04X", addrToRead[i]);
        std::snprintf(value, sizeof(value), "0x%04X", dataReceived[i]);
        fout << addr << "="sv << value << std::endl;
    }

    fout << "[reference_clocks]"sv << std::endl;
    fout << "sxt_ref_clk_mhz="sv << this->GetReferenceClk_SX(TRXDir::Tx) / 1e6 << std::endl;
    fout << "sxr_ref_clk_mhz="sv << this->GetReferenceClk_SX(TRXDir::Rx) / 1e6 << std::endl;
    fout.close();
    return OpStatus::Success;
}

OpStatus LMS7002M::SetRBBPGA_dB(const float_type value, const Channel channel)
{
    lime_Result result = lms7002m_set_rbbpga_db(mC_impl, value, static_cast<lms7002m_channel>(channel));
    return ResultToStatus(result);
}

float_type LMS7002M::GetRBBPGA_dB(const Channel channel)
{
    return lms7002m_get_rbbpga_db(mC_impl, static_cast<lms7002m_channel>(channel));
}

OpStatus LMS7002M::SetRFELNA_dB(const float_type value, const Channel channel)
{
    lime_Result result = lms7002m_set_rfelna_db(mC_impl, value, static_cast<lms7002m_channel>(channel));
    return ResultToStatus(result);
}

float_type LMS7002M::GetRFELNA_dB(const Channel channel)
{
    return lms7002m_get_rfelna_db(mC_impl, static_cast<lms7002m_channel>(channel));
}

OpStatus LMS7002M::SetRFELoopbackLNA_dB(const float_type gain, const Channel channel)
{
    lime_Result result = lms7002m_set_rfe_loopback_lna_db(mC_impl, gain, static_cast<lms7002m_channel>(channel));
    return ResultToStatus(result);
}

float_type LMS7002M::GetRFELoopbackLNA_dB(const Channel channel)
{
    return lms7002m_get_rfe_loopback_lna_db(mC_impl, static_cast<lms7002m_channel>(channel));
}

OpStatus LMS7002M::SetRFETIA_dB(const float_type value, const Channel channel)
{
    lime_Result result = lms7002m_set_rfetia_db(mC_impl, value, static_cast<lms7002m_channel>(channel));
    return ResultToStatus(result);
}

float_type LMS7002M::GetRFETIA_dB(const Channel channel)
{
    return lms7002m_get_rfetia_db(mC_impl, static_cast<lms7002m_channel>(channel));
}

OpStatus LMS7002M::SetTRFPAD_dB(const float_type value, const Channel channel)
{
    lime_Result result = lms7002m_set_trfpad_db(mC_impl, value, static_cast<lms7002m_channel>(channel));
    return ResultToStatus(result);
}

float_type LMS7002M::GetTRFPAD_dB(const Channel channel)
{
    return lms7002m_get_trfpad_db(mC_impl, static_cast<lms7002m_channel>(channel));
}

OpStatus LMS7002M::SetTRFLoopbackPAD_dB(const float_type gain, const Channel channel)
{
    lime_Result result = lms7002m_set_trf_loopback_pad_db(mC_impl, gain, static_cast<lms7002m_channel>(channel));
    return ResultToStatus(result);
}

float_type LMS7002M::GetTRFLoopbackPAD_dB(const Channel channel)
{
    return lms7002m_get_trf_loopback_pad_db(mC_impl, static_cast<lms7002m_channel>(channel));
}

// opt_gain_tbb
OpStatus LMS7002M::SetTBBIAMP_dB(const float_type gain, const Channel channel)
{
    OpStatus status;
    ChannelScope scope(this, channel);

    int ind = this->GetActiveChannelIndex() % 2;
    if (opt_gain_tbb[ind] <= 0)
    {
        status = CalibrateTxGain();
        if (status != OpStatus::Success) //set optimal BB gain
            return status;
        if (std::fabs(gain) < 0.2) // optimal gain = ~0dB
            return OpStatus::Success;
    }

    int g_iamp = static_cast<float_type>(opt_gain_tbb[ind]) * pow(10.0, gain / 20.0) + 0.4;
    status = Modify_SPI_Reg_bits(LMS7002MCSR::CG_IAMP_TBB, std::clamp(g_iamp, 1, 63), true);

    return status;
}

// opt_gain_tbb
float_type LMS7002M::GetTBBIAMP_dB(const Channel channel)
{
    ChannelScope scope(this, channel);

    int g_current = Get_SPI_Reg_bits(LMS7002MCSR::CG_IAMP_TBB, true);
    int ind = this->GetActiveChannelIndex() % 2;

    if (opt_gain_tbb[ind] <= 0)
    {
        if (CalibrateTxGain() != OpStatus::Success)
            return 0.0;
        Modify_SPI_Reg_bits(LMS7002MCSR::CG_IAMP_TBB, g_current, true); //restore
    }
    return 20.0 * log10(static_cast<float_type>(g_current) / static_cast<float_type>(opt_gain_tbb[ind]));
}

OpStatus LMS7002M::SetPathRFE(PathRFE path)
{
    lime_Result result = lms7002m_set_path_rfe(mC_impl, static_cast<lms7002m_path_rfe>(path));
    return ResultToStatus(result);
}

LMS7002M::PathRFE LMS7002M::GetPathRFE()
{
    return static_cast<PathRFE>(lms7002m_get_path_rfe(mC_impl));
}

OpStatus LMS7002M::SetBandTRF(const int band)
{
    lime_Result result = lms7002m_set_band_trf(mC_impl, static_cast<uint8_t>(band));
    return ResultToStatus(result);
}

int LMS7002M::GetBandTRF()
{
    return lms7002m_get_band_trf(mC_impl);
}

OpStatus LMS7002M::SetPath(TRXDir direction, uint8_t channel, uint8_t path)
{
    lime_Result result = lms7002m_set_path(mC_impl, direction == TRXDir::Tx, static_cast<lms7002m_channel>(channel), path);
    return ResultToStatus(result);
}

OpStatus LMS7002M::SetReferenceClk_SX(TRXDir dir, float_type freq_Hz)
{
    lime_Result result = lms7002m_set_reference_clock(mC_impl, freq_Hz);
    return ResultToStatus(result);
}

float_type LMS7002M::GetReferenceClk_SX(TRXDir dir)
{
    return lms7002m_get_reference_clock(mC_impl);
}

OpStatus LMS7002M::SetNCOFrequencies(TRXDir dir, const float_type* freq_Hz, uint8_t count, float_type phaseOffset)
{
    lime_Result result = lms7002m_set_nco_frequencies(mC_impl, dir == TRXDir::Tx, freq_Hz, count, phaseOffset);
    return ResultToStatus(result);
}

std::vector<float_type> LMS7002M::GetNCOFrequencies(TRXDir dir, float_type* phaseOffset)
{
    std::vector<float_type> ncos(16);
    lms7002m_get_nco_frequencies(mC_impl, dir == TRXDir::Tx, ncos.data(), ncos.size(), phaseOffset);
    return ncos;
}

float_type LMS7002M::GetFrequencyCGEN()
{
    return lms7002m_get_frequency_cgen(mC_impl);
}

float_type LMS7002M::GetReferenceClk_TSP(TRXDir dir)
{
    return lms7002m_get_reference_clock_tsp(mC_impl, dir == TRXDir::Tx);
}

OpStatus LMS7002M::SetFrequencyCGEN(const float_type freq_Hz, const bool retainNCOfrequencies, CGEN_details* output)
{
    lime_Result result = lms7002m_set_frequency_cgen(mC_impl, freq_Hz);
    return ResultToStatus(result);
}

bool LMS7002M::GetCGENLocked()
{
    return lms7002m_get_cgen_locked(mC_impl);
}

bool LMS7002M::GetSXLocked(TRXDir dir)
{
    return lms7002m_get_sx_locked(mC_impl, dir == TRXDir::Tx);
}

OpStatus LMS7002M::TuneCGENVCO()
{
    lime_Result result = lms7002m_tune_cgen_vco(mC_impl);
    return ResultToStatus(result);
}

OpStatus LMS7002M::TuneVCO(VCO_Module module) // 0-cgen, 1-SXR, 2-SXT
{
    lime_Result result = lms7002m_tune_vco(mC_impl, static_cast<lms7002m_vco_type>(module));
    return ResultToStatus(result);
}

uint16_t LMS7002M::Get_SPI_Reg_bits(const CSRegister& param, bool fromChip)
{
    return Get_SPI_Reg_bits(param.address, param.msb, param.lsb, fromChip);
}

uint16_t LMS7002M::Get_SPI_Reg_bits(uint16_t address, uint8_t msb, uint8_t lsb, bool fromChip)
{
    return (SPI_read(address, fromChip) & (~(~0u << (msb + 1)))) >> lsb; //shift bits to LSB
}

uint16_t LMS7002M::Get_SPI_Reg_bits(const LMS7002MCSR param, bool fromChip)
{
    const CSRegister& reg = GetRegister(param);
    return Get_SPI_Reg_bits(reg.address, reg.msb, reg.lsb, fromChip);
}

OpStatus LMS7002M::Modify_SPI_Reg_bits(
    const uint16_t address, const uint8_t msb, const uint8_t lsb, const uint16_t value, bool fromChip)
{
    uint16_t spiDataReg = SPI_read(address, fromChip); //read current SPI reg data
    uint16_t spiMask = (~(~0u << (msb - lsb + 1))) << (lsb); // creates bit mask
    spiDataReg = (spiDataReg & (~spiMask)) | ((value << lsb) & spiMask); //clear bits
    return SPI_write(address, spiDataReg); //write modified data back to SPI reg
}

OpStatus LMS7002M::Modify_SPI_Reg_bits(const LMS7002MCSR_Data::CSRegister& param, const uint16_t value, bool fromChip)
{
    return Modify_SPI_Reg_bits(param.address, param.msb, param.lsb, value, fromChip);
}

OpStatus LMS7002M::Modify_SPI_Reg_bits(const LMS7002MCSR param, const uint16_t value, bool fromChip)
{
    const CSRegister& reg = GetRegister(param);
    return Modify_SPI_Reg_bits(reg, value, fromChip);
}

const CSRegister& LMS7002M::GetParam(const std::string& name)
{
    for (int i = 0; i < static_cast<int>(LMS7002MCSR::ENUM_COUNT); ++i)
    {
        const LMS7002MCSR_Data::CSRegister& parameter = GetRegister(static_cast<LMS7002MCSR>(i));
        if (std::string_view{ parameter.name } == name)
            return parameter;
    }

    throw std::logic_error("Parameter "s + name + " not found"s);
}

OpStatus LMS7002M::SetFrequencySX(TRXDir dir, float_type freq_Hz, SX_details* output)
{
    static std::map<float_type, int8_t> tuning_cache_sel_vco;
    static std::map<float_type, int16_t> tuning_cache_csw_value;

    assert(freq_Hz > 0);

    const char* vcoNames[] = { "VCOL", "VCOM", "VCOH" };
    const uint8_t sxVCO_N = 2; //number of entries in VCO frequencies
    const float_type m_dThrF = 5500e6; //threshold to enable additional divider
    float_type VCOfreq;
    int8_t div_loch;
    int8_t sel_vco;
    bool canDeliverFrequency = false;
    uint16_t integerPart;
    uint32_t fractionalPart;
    int16_t csw_value;

    //find required VCO frequency
    for (div_loch = 6; div_loch >= 0; --div_loch)
    {
        VCOfreq = (1 << (div_loch + 1)) * freq_Hz;
        if ((VCOfreq >= gVCO_frequency_table[0][0]) && (VCOfreq <= gVCO_frequency_table[2][sxVCO_N - 1]))
        {
            canDeliverFrequency = true;
            break;
        }
    }
    if (canDeliverFrequency == false)
        return ReportError(OpStatus::OutOfRange,
            "SetFrequencySX%s(%g MHz) - required VCO frequencies are out of range [%g-%g] MHz",
            dir == TRXDir::Tx ? "T" : "R",
            freq_Hz / 1e6,
            gVCO_frequency_table[0][0] / 1e6,
            gVCO_frequency_table[2][sxVCO_N - 1] / 1e6);

    const float_type refClk_Hz = GetReferenceClk_SX(dir);
    assert(refClk_Hz > 0);
    double divider = refClk_Hz * (1 + (VCOfreq > m_dThrF));
    integerPart = static_cast<uint16_t>(VCOfreq / divider - 4);
    fractionalPart = static_cast<uint32_t>((VCOfreq / divider - static_cast<uint32_t>(VCOfreq / divider)) * 1048576);

    ChannelScope scope(this);
    this->SetActiveChannel(dir == TRXDir::Tx ? Channel::ChSXT : Channel::ChSXR);

    Modify_SPI_Reg_bits(LMS7002MCSR::EN_INTONLY_SDM, 0);
    Modify_SPI_Reg_bits(LMS7002MCSR::INT_SDM, integerPart); //INT_SDM
    Modify_SPI_Reg_bits(0x011D, 15, 0, fractionalPart & 0xFFFF); //FRAC_SDM[15:0]
    Modify_SPI_Reg_bits(0x011E, 3, 0, (fractionalPart >> 16)); //FRAC_SDM[19:16]
    Modify_SPI_Reg_bits(LMS7002MCSR::DIV_LOCH, div_loch); //DIV_LOCH
    Modify_SPI_Reg_bits(LMS7002MCSR::EN_DIV2_DIVPROG, (VCOfreq > m_dThrF)); //EN_DIV2_DIVPROG

    lime::info("SetFrequencySX%s, (%.3f MHz)INT %d, FRAC %d, DIV_LOCH %d, EN_DIV2_DIVPROG %d",
        dir == TRXDir::Tx ? "T" : "R",
        freq_Hz / 1e6,
        integerPart,
        fractionalPart,
        div_loch,
        (VCOfreq > m_dThrF));
    lime::debug("Expected VCO %.2f MHz, RefClk %.2f MHz", VCOfreq / 1e6, refClk_Hz / 1e6);

    if (output)
    {
        output->frequency = freq_Hz;
        output->frequencyVCO = VCOfreq;
        output->referenceClock = GetReferenceClk_SX(dir);
        output->INT = integerPart;
        output->FRAC = fractionalPart;
        output->en_div2_divprog = (VCOfreq > m_dThrF);
        output->div_loch = div_loch;
    }

    // turn on VCO and comparator
    Modify_SPI_Reg_bits(LMS7002MCSR::PD_VCO, 0); //
    Modify_SPI_Reg_bits(LMS7002MCSR::PD_VCO_COMP, 0);

    // try setting tuning values from the cache, if it fails perform full tuning
    if (useCache && tuning_cache_sel_vco.count(freq_Hz) > 0)
    {
        sel_vco = tuning_cache_sel_vco[freq_Hz];
        csw_value = tuning_cache_csw_value[freq_Hz];
        Modify_SPI_Reg_bits(LMS7002MCSR::SEL_VCO, sel_vco);
        Modify_SPI_Reg_bits(CSW_VCO.address, CSW_VCO.msb, CSW_VCO.lsb, csw_value);
        // probably no need for this as the interface is already very slow..
        std::this_thread::sleep_for(std::chrono::microseconds(50));
        auto cmphl = static_cast<uint8_t>(Get_SPI_Reg_bits(VCO_CMPHO.address, 13, 12, true));
        if (cmphl == 2)
        {
            lime::info("Fast Tune success; vco=%d value=%d", tuning_cache_sel_vco[freq_Hz], tuning_cache_csw_value[freq_Hz]);
            if (output)
            {
                output->success = true;
                output->sel_vco = sel_vco;
                output->csw = csw_value;
            }
            return OpStatus::Success;
        }
    }

    canDeliverFrequency = false;
    int tuneScore[] = { -128, -128, -128 }; //best is closest to 0
    for (int i = 0; i < 5; i++) //attempt tune multiple times
    {
        for (sel_vco = 0; sel_vco < 3; ++sel_vco)
        {
            lime::debug("Tuning %s :", vcoNames[sel_vco]);
            Modify_SPI_Reg_bits(LMS7002MCSR::SEL_VCO, sel_vco);
            OpStatus status = TuneVCO(dir == TRXDir::Tx ? VCO_Module::VCO_SXT : VCO_Module::VCO_SXR);
            if (status == OpStatus::Success)
            {
                tuneScore[sel_vco] = -128 + Get_SPI_Reg_bits(LMS7002MCSR::CSW_VCO, true);
                canDeliverFrequency = true;
                lime::debug("%s : csw=%d %s",
                    vcoNames[sel_vco],
                    tuneScore[sel_vco] + 128,
                    (status == OpStatus::Success ? "tune ok" : "tune fail"));
            }
            else
            {
                lime::debug("%s : failed to lock", vcoNames[sel_vco]);
            }
        }
        if (canDeliverFrequency) //tune OK
            break;
        auto bias = Get_SPI_Reg_bits(LMS7002MCSR::ICT_VCO);
        if (bias == 255)
            break;
        bias = bias + 32 > 255 ? 255 : bias + 32; //retry with higher bias current
        Modify_SPI_Reg_bits(LMS7002MCSR::ICT_VCO, bias);
    }

    if (abs(tuneScore[0]) < abs(tuneScore[1]))
    {
        if (abs(tuneScore[0]) < abs(tuneScore[2]))
            sel_vco = 0;
        else
            sel_vco = 2;
    }
    else
    {
        if (abs(tuneScore[1]) < abs(tuneScore[2]))
            sel_vco = 1;
        else
            sel_vco = 2;
    }
    csw_value = tuneScore[sel_vco] + 128;
    lime::debug("Selected: %s, CSW_VCO: %i", vcoNames[sel_vco], csw_value);

    if (output)
    {
        if (canDeliverFrequency)
            output->success = true;
        output->sel_vco = sel_vco;
        output->csw = csw_value;
    }
    Modify_SPI_Reg_bits(LMS7002MCSR::SEL_VCO, sel_vco);
    Modify_SPI_Reg_bits(LMS7002MCSR::CSW_VCO, csw_value);

    // save successful tuning results in cache
    if (useCache && canDeliverFrequency)
    {
        tuning_cache_sel_vco[freq_Hz] = sel_vco;
        tuning_cache_csw_value[freq_Hz] = csw_value;
    }

    if (canDeliverFrequency == false)
        return ReportError(
            OpStatus::Error, "SetFrequencySX%s(%g MHz) - cannot deliver frequency", dir == TRXDir::Tx ? "T" : "R", freq_Hz / 1e6);
    return OpStatus::Success;
}

OpStatus LMS7002M::SetFrequencySXWithSpurCancelation(TRXDir dir, float_type freq_Hz, float_type BW)
{
    const float BWOffset = 2e6;
    BW += BWOffset; //offset to avoid ref clock on BW edge
    bool needCancelation = false;
    float_type refClk = GetReferenceClk_SX(TRXDir::Rx);
    int low = (freq_Hz - BW / 2) / refClk;
    int high = (freq_Hz + BW / 2) / refClk;
    if (low != high)
        needCancelation = true;

    OpStatus status;
    float newFreq(0);
    if (needCancelation)
    {
        newFreq = static_cast<int>(freq_Hz / refClk + 0.5) * refClk;
        TuneRxFilter(BW - BWOffset + 2 * abs(freq_Hz - newFreq));
        status = SetFrequencySX(dir, newFreq);
    }
    else
        status = SetFrequencySX(dir, freq_Hz);
    if (status != OpStatus::Success)
        return status;
    const int ch = Get_SPI_Reg_bits(LMS7002MCSR::MAC);
    for (int i = 0; i < 2; ++i)
    {
        Modify_SPI_Reg_bits(LMS7002MCSR::MAC, i + 1);
        SetNCOFrequency(TRXDir::Rx, 15, 0);
    }
    if (needCancelation)
    {
        Modify_SPI_Reg_bits(LMS7002MCSR::MAC, ch);
        Modify_SPI_Reg_bits(LMS7002MCSR::EN_INTONLY_SDM, 1);

        /*uint16_t gINT = Get_SPI_Reg_bits(0x011E, 13, 0);	// read whole register to reduce SPI transfers
        uint32_t gFRAC = ((gINT&0xF) * 65536) | Get_SPI_Reg_bits(0x011D, 15, 0);
        bool upconvert = gFRAC > (1 << 19);
        gINT = gINT >> 4;
        if(upconvert)
        {
            gINT+=;
            Modify_SPI_Reg_bits(LMS7002MCSR::INT_SDM, gINT);
        }
        Modify_SPI_Reg_bits(0x011D, 15, 0, 0);
        Modify_SPI_Reg_bits(0x011E, 3, 0, 0);*/
        //const float_type refClk_Hz = GetReferenceClk_SX(dir);
        //float actualFreq = (float_type)refClk_Hz / (1 << (Get_SPI_Reg_bits(LMS7002MCSR::DIV_LOCH) + 1));
        //actualFreq *= (gINT + 4) * (Get_SPI_Reg_bits(LMS7002MCSR::EN_DIV2_DIVPROG) + 1);
        float actualFreq = newFreq;
        float userFreq = freq_Hz;
        bool upconvert = actualFreq > userFreq;
        for (int i = 0; i < 2; ++i)
        {
            Modify_SPI_Reg_bits(LMS7002MCSR::MAC, i + 1);
            Modify_SPI_Reg_bits(LMS7002MCSR::CMIX_SC_RXTSP, !upconvert);
            Modify_SPI_Reg_bits(LMS7002MCSR::CMIX_BYP_RXTSP, 0);
            Modify_SPI_Reg_bits(LMS7002MCSR::SEL_RX, 15);
            Modify_SPI_Reg_bits(LMS7002MCSR::CMIX_GAIN_RXTSP, 1);
            SetNCOFrequency(TRXDir::Rx, 14, 0);
            SetNCOFrequency(TRXDir::Rx, 15, abs(actualFreq - userFreq));
        }
    }

    Modify_SPI_Reg_bits(LMS7002MCSR::MAC, ch);
    return OpStatus::Success;
}

float_type LMS7002M::GetFrequencySX(TRXDir dir)
{
    return lms7002m_get_frequency_sx(mC_impl, dir == TRXDir::Tx);
}

OpStatus LMS7002M::SetNCOFrequency(TRXDir dir, uint8_t index, float_type freq_Hz)
{
    lime_Result result = lms7002m_set_nco_frequency(mC_impl, dir == TRXDir::Tx, index, freq_Hz);
    return ResultToStatus(result);
}

float_type LMS7002M::GetNCOFrequency(TRXDir dir, uint8_t index, bool fromChip)
{
    return lms7002m_get_nco_frequency(mC_impl, dir == TRXDir::Tx, index);
}

OpStatus LMS7002M::SetNCOPhaseOffsetForMode0(TRXDir dir, float_type angle_deg)
{
    lime_Result result = lms7002m_set_nco_phase_offset_for_mode_0(mC_impl, dir == TRXDir::Tx, angle_deg);
    return ResultToStatus(result);
}

OpStatus LMS7002M::SetNCOPhaseOffset(TRXDir dir, uint8_t index, float_type angle_deg)
{
    lime_Result result = lms7002m_set_nco_phase_offset(mC_impl, dir == TRXDir::Tx, index, angle_deg);
    return ResultToStatus(result);
}

OpStatus LMS7002M::SetNCOPhases(TRXDir dir, const float_type* angles_deg, uint8_t count, float_type frequencyOffset)
{
    lime_Result result = lms7002m_set_nco_phases(mC_impl, dir == TRXDir::Tx, angles_deg, count, frequencyOffset);
    return ResultToStatus(result);
}

std::vector<float_type> LMS7002M::GetNCOPhases(TRXDir dir, float_type* frequencyOffset)
{
    std::vector<float_type> angles_deg;
    return angles_deg;
}

OpStatus LMS7002M::SetGFIRCoefficients(TRXDir dir, uint8_t gfirIndex, const float_type* coef, uint8_t coefCount)
{
    lime_Result result = lms7002m_set_gfir_coefficients(mC_impl, dir == TRXDir::Tx, gfirIndex, coef, coefCount);
    return ResultToStatus(result);
}

OpStatus LMS7002M::GetGFIRCoefficients(TRXDir dir, uint8_t gfirIndex, float_type* coef, uint8_t coefCount)
{
    lime_Result result = lms7002m_get_gfir_coefficients(mC_impl, dir == TRXDir::Tx, gfirIndex, coef, coefCount);
    return ResultToStatus(result);
}

OpStatus LMS7002M::SPI_write(uint16_t address, uint16_t data, bool toChip)
{
    if (address != 0x0640 && address != 0x0641)
    {
        return SPI_write_batch(&address, &data, 1, toChip);
    }

    MCU_BD* mcu = GetMCUControls();
    mcu->RunProcedure(MCU_FUNCTION_GET_PROGRAM_ID);
    if (mcu->WaitForMCU(100) != MCU_ID_CALIBRATIONS_SINGLE_IMAGE)
        mcu->Program_MCU(mcu_program_lms7_dc_iq_calibration_bin, MCU_BD::MCU_PROG_MODE::SRAM);
    SPI_write(0x002D, address);
    SPI_write(0x020C, data);
    mcu->RunProcedure(7);
    mcu->WaitForMCU(50);
    return SPI_read(0x040B) == data ? OpStatus::Success : OpStatus::Error;
}

uint16_t LMS7002M::SPI_read(uint16_t address, bool fromChip, OpStatus* status)
{
    fromChip |= !useCache;
    //registers containing read only registers, which values can change
    static const std::unordered_set<uint16_t> volatileRegs = {
        0x0000,
        0x0001,
        0x0002,
        0x0003,
        0x0004,
        0x0005,
        0x0006,
        0x002F,
        0x008C,
        0x00A8,
        0x00A9,
        0x00AA,
        0x00AB,
        0x00AC,
        0x0123,
        0x0209,
        0x020A,
        0x020B,
        0x040E,
        0x040F,
        0x05C3,
        0x05C4,
        0x05C5,
        0x05C6,
        0x05C7,
        0x05C8,
        0x05C9,
        0x05CA,
    };
    if (volatileRegs.find(address) != volatileRegs.end())
        fromChip = true;

    if (!controlPort || fromChip == false)
    {
        if (status && !controlPort)
            *status = ReportError(OpStatus::IOFailure, "chip not connected");
        uint8_t mac = mRegistersMap->GetValue(0, MAC.address) & 0x0003;
        uint8_t channel = (mac == 2) ? 1 : 0; //only when MAC is B -> use register space B
        if (address < 0x0100)
            channel = 0; //force A when below MAC mapped register space
        return mRegistersMap->GetValue(channel, address);
    }
    if (!controlPort)
    {
        return 0;
    }

    uint16_t data = 0;
    OpStatus st;
    if (address == 0x0640 || address == 0x0641)
    {
        MCU_BD* mcu = GetMCUControls();
        mcu->RunProcedure(MCU_FUNCTION_GET_PROGRAM_ID);
        if (mcu->WaitForMCU(100) != MCU_ID_CALIBRATIONS_SINGLE_IMAGE)
            mcu->Program_MCU(mcu_program_lms7_dc_iq_calibration_bin, MCU_BD::MCU_PROG_MODE::SRAM);
        SPI_write(0x002D, address);
        mcu->RunProcedure(8);
        mcu->WaitForMCU(50);
        uint16_t rdVal = SPI_read(0x040B, true, status);
        return rdVal;
    }
    else
        st = this->SPI_read_batch(&address, &data, 1);
    if (status != nullptr)
        *status = st;
    return data;
}

OpStatus LMS7002M::SPI_write_batch(const uint16_t* spiAddr, const uint16_t* spiData, uint16_t cnt, bool toChip)
{
    toChip |= !useCache;
    int mac = mRegistersMap->GetValue(0, MAC.address) & 0x0003;
    std::vector<uint32_t> data;
    for (size_t i = 0; i < cnt; ++i)
    {
        //write which register cache based on MAC bits
        //or always when below the MAC mapped register space
        bool wr0 = ((mac & 0x1) != 0) || (spiAddr[i] < 0x0100);
        bool wr1 = ((mac & 0x2) != 0) && (spiAddr[i] >= 0x0100);

        if (!toChip)
        {
            if (wr0 && (mRegistersMap->GetValue(0, spiAddr[i]) == spiData[i]))
                wr0 = false;
            if (wr1 && (mRegistersMap->GetValue(1, spiAddr[i]) == spiData[i]))
                wr1 = false;
            if (!(wr0 || wr1))
                continue;
        }

        data.push_back((1 << 31) | (static_cast<uint32_t>(spiAddr[i]) << 16) | spiData[i]); //msbit 1=SPI write
        if (wr0)
            mRegistersMap->SetValue(0, spiAddr[i], spiData[i]);
        if (wr1)
            mRegistersMap->SetValue(1, spiAddr[i], spiData[i]);

        //refresh mac, because batch might also change active channel
        if (spiAddr[i] == MAC.address)
            mac = mRegistersMap->GetValue(0, MAC.address) & 0x0003;
    }

    if (data.size() == 0)
        return OpStatus::Success;
    if (!controlPort)
    {
        if (useCache)
            return OpStatus::Success;
        return ReportError(OpStatus::IOFailure, "No device connected"s);
    }
    controlPort->SPI(data.data(), nullptr, data.size());
    return OpStatus::Success;
}

OpStatus LMS7002M::SPI_read_batch(const uint16_t* spiAddr, uint16_t* spiData, uint16_t cnt)
{
    if (!controlPort)
    {
        return ReportError(OpStatus::IOFailure, "No device connected"s);
    }

    std::vector<uint32_t> dataWr(cnt);
    std::vector<uint32_t> dataRd(cnt);
    for (size_t i = 0; i < cnt; ++i)
    {
        dataWr[i] = spiAddr[i];
    }

    controlPort->SPI(dataWr.data(), dataRd.data(), cnt);

    int mac = mRegistersMap->GetValue(0, MAC.address) & 0x0003;

    for (size_t i = 0; i < cnt; ++i)
    {
        spiData[i] = dataRd[i] & 0xffff;

        //write which register cache based on MAC bits
        //or always when below the MAC mapped register space
        bool wr0 = ((mac & 0x1) != 0) or (spiAddr[i] < 0x0100);
        bool wr1 = ((mac & 0x2) != 0) and (spiAddr[i] >= 0x0100);

        if (wr0)
            mRegistersMap->SetValue(0, spiAddr[i], spiData[i]);
        if (wr1)
            mRegistersMap->SetValue(1, spiAddr[i], spiData[i]);
    }
    return OpStatus::Success;
}

OpStatus LMS7002M::RegistersTest(const std::string& fileName)
{
    char chex[16];
    if (!controlPort)
        return ReportError(OpStatus::IOFailure, "No device connected"s);

    OpStatus status;
    ChannelScope scope(this);

    //backup both channel data for restoration after test
    std::vector<uint16_t> ch1Addresses;
    for (const auto& memorySectionPair : MemorySectionAddresses)
        for (uint16_t addr = memorySectionPair.second[0]; addr <= memorySectionPair.second[1]; ++addr)
            ch1Addresses.push_back(addr);
    std::vector<uint16_t> ch1Data;
    ch1Data.resize(ch1Addresses.size(), 0);

    //backup A channel
    this->SetActiveChannel(Channel::ChA);
    status = SPI_read_batch(&ch1Addresses[0], &ch1Data[0], ch1Addresses.size());
    if (status != OpStatus::Success)
        return status;

    std::vector<uint16_t> ch2Addresses;
    for (const auto& memorySectionPair : MemorySectionAddresses)
        for (uint16_t addr = memorySectionPair.second[0]; addr <= memorySectionPair.second[1]; ++addr)
            if (addr >= 0x0100)
                ch2Addresses.push_back(addr);
    std::vector<uint16_t> ch2Data;
    ch2Data.resize(ch2Addresses.size(), 0);

    this->SetActiveChannel(Channel::ChB);
    status = SPI_read_batch(&ch2Addresses[0], &ch2Data[0], ch2Addresses.size());
    if (status != OpStatus::Success)
        return status;

    //test registers
    ResetChip();
    Modify_SPI_Reg_bits(LMS7002MCSR::MIMO_SISO, 0);
    Modify_SPI_Reg_bits(LMS7002MCSR::PD_RX_AFE2, 0);
    Modify_SPI_Reg_bits(LMS7002MCSR::PD_TX_AFE2, 0);
    this->SetActiveChannel(Channel::ChA);

    std::stringstream ss;

    //check single channel memory sections
    std::vector<MemorySection> modulesToCheck = {
        MemorySection::AFE,
        MemorySection::BIAS,
        MemorySection::XBUF,
        MemorySection::CGEN,
        MemorySection::BIST,
        MemorySection::CDS,
        MemorySection::TRF,
        MemorySection::TBB,
        MemorySection::RFE,
        MemorySection::RBB,
        MemorySection::SX,
        MemorySection::TxTSP,
        MemorySection::TxNCO,
        MemorySection::TxGFIR1,
        MemorySection::TxGFIR2,
        MemorySection::TxGFIR3a,
        MemorySection::TxGFIR3b,
        MemorySection::TxGFIR3c,
        MemorySection::RxTSP,
        MemorySection::RxNCO,
        MemorySection::RxGFIR1,
        MemorySection::RxGFIR2,
        MemorySection::RxGFIR3a,
        MemorySection::RxGFIR3b,
        MemorySection::RxGFIR3c,
        MemorySection::LimeLight,
        MemorySection::LDO,
    };

    const std::array<std::string_view, 27> moduleNames = {
        "AFE"sv,
        "BIAS"sv,
        "XBUF"sv,
        "CGEN"sv,
        "BIST"sv,
        "CDS"sv,
        "TRF"sv,
        "TBB"sv,
        "RFE"sv,
        "RBB"sv,
        "SX"sv,
        "TxTSP"sv,
        "TxNCO"sv,
        "TxGFIR1"sv,
        "TxGFIR2"sv,
        "TxGFIR3a"sv,
        "TxGFIR3b"sv,
        "TxGFIR3c"sv,
        "RxTSP"sv,
        "RxNCO"sv,
        "RxGFIR1"sv,
        "RxGFIR2"sv,
        "RxGFIR3a"sv,
        "RxGFIR3b"sv,
        "RxGFIR3c"sv,
        "LimeLight"sv,
        "LDO"sv,
    };

    const uint16_t patterns[] = { 0xAAAA, 0x5555 };
    const uint8_t patternsCount = 2;

    bool allTestSuccess = true;

    for (unsigned i = 0; i < modulesToCheck.size(); ++i)
    {
        bool moduleTestsSuccess = true;
        uint16_t startAddr = MemorySectionAddresses.at(modulesToCheck[i]).at(0);
        uint16_t endAddr = MemorySectionAddresses.at(modulesToCheck[i]).at(1);
        uint8_t channelCount = startAddr >= 0x0100 ? 2 : 1;
        for (int cc = 1; cc <= channelCount; ++cc)
        {
            Modify_SPI_Reg_bits(LMS7002MCSR::MAC, cc);
            std::snprintf(chex, sizeof(chex), "0x%04X", startAddr);
            ss << moduleNames[i] << "  ["sv << chex << ":"sv;
            std::snprintf(chex, sizeof(chex), "0x%04X", endAddr);
            ss << chex << "]"sv;
            if (startAddr >= 0x0100)
            {
                ss << " Ch."sv << (cc == 1 ? "A"sv : "B"sv);
            }
            ss << std::endl;
            for (uint8_t p = 0; p < patternsCount; ++p)
                moduleTestsSuccess &= RegistersTestInterval(startAddr, endAddr, patterns[p], ss) == OpStatus::Success;
        }
        allTestSuccess &= moduleTestsSuccess;
    }

    //restore register values
    this->SetActiveChannel(Channel::ChA);
    SPI_write_batch(&ch1Addresses[0], &ch1Data[0], ch1Addresses.size(), true);
    this->SetActiveChannel(Channel::ChB);
    SPI_write_batch(&ch2Addresses[0], &ch2Data[0], ch2Addresses.size(), true);

    if (!fileName.empty())
    {
        std::fstream fout;
        fout.open(fileName, std::ios::out);
        fout << ss.str() << std::endl;
        fout.close();
    }

    if (allTestSuccess)
        return OpStatus::Success;
    return ReportError(OpStatus::Error, "RegistersTest() failed"s);
}

/** @brief Performs registers test for given address interval by writing given pattern data
    @param startAddr first register address
    @param endAddr last reigster address
    @param pattern data to be written into registers
    @param ss stringstream to use
    @return 0-register test passed, other-failure
*/
OpStatus LMS7002M::RegistersTestInterval(uint16_t startAddr, uint16_t endAddr, uint16_t pattern, std::stringstream& ss)
{
    std::vector<uint16_t> addrToWrite;
    std::vector<uint16_t> dataToWrite;
    std::vector<uint16_t> dataReceived;
    std::vector<uint16_t> dataMasks;

    for (uint16_t addr = startAddr; addr <= endAddr; ++addr)
    {
        addrToWrite.push_back(addr);
    }
    dataMasks.resize(addrToWrite.size(), 0xFFFF);
    for (std::size_t j = 0; j < readOnlyRegisters.size(); ++j)
    {
        for (std::size_t k = 0; k < addrToWrite.size(); ++k)
        {
            if (readOnlyRegisters[j].address == addrToWrite[k])
            {
                dataMasks[k] = readOnlyRegisters[j].mask;
                break;
            }
        }
    }

    dataToWrite.clear();
    dataReceived.clear();
    for (uint16_t j = 0; j < addrToWrite.size(); ++j)
    {
        if (addrToWrite[j] == 0x00A6)
            dataToWrite.push_back(0x1 | (pattern & ~0x2));
        else if (addrToWrite[j] == 0x0084)
            dataToWrite.push_back(pattern & ~0x19);
        else
            dataToWrite.push_back(pattern & dataMasks[j]);
    }
    dataReceived.resize(addrToWrite.size(), 0);
    OpStatus status;
    status = SPI_write_batch(&addrToWrite[0], &dataToWrite[0], addrToWrite.size(), true);
    if (status != OpStatus::Success)
        return status;
    status = SPI_read_batch(&addrToWrite[0], &dataReceived[0], addrToWrite.size());
    if (status != OpStatus::Success)
        return status;
    bool registersMatch = true;
    char ctemp[16];
    for (uint16_t j = 0; j < dataToWrite.size(); ++j)
    {
        if (dataToWrite[j] != (dataReceived[j] & dataMasks[j]))
        {
            registersMatch = false;
            std::snprintf(ctemp, sizeof(ctemp), "0x%04X", addrToWrite[j]);
            ss << "\t"sv << ctemp << "(wr/rd): "sv;
            std::snprintf(ctemp, sizeof(ctemp), "0x%04X", dataToWrite[j]);
            ss << ctemp << "/"sv;
            std::snprintf(ctemp, sizeof(ctemp), "0x%04X", dataReceived[j]);
            ss << ctemp << std::endl;
        }
    }
    if (registersMatch)
    {
        std::snprintf(ctemp, sizeof(ctemp), "0x%04X", pattern);
        ss << "\tRegisters OK ("sv << ctemp << ")\n"sv;
    }
    if (registersMatch)
        return OpStatus::Success;
    return ReportError(OpStatus::Error, "RegistersTestInterval(startAddr=0x%x, endAddr=0x%x) - failed", startAddr, endAddr);
}

OpStatus LMS7002M::SetDefaults(MemorySection module)
{
    OpStatus status;
    std::vector<uint16_t> addrs;
    std::vector<uint16_t> values;
    for (uint16_t address = MemorySectionAddresses.at(module).at(0); address <= MemorySectionAddresses.at(module).at(1); ++address)
    {
        addrs.push_back(address);
        values.push_back(mRegistersMap->GetDefaultValue(address));
    }
    status = SPI_write_batch(&addrs[0], &values[0], addrs.size());
    return status;
}

void LMS7002M::ModifyRegistersDefaults(const std::vector<std::pair<uint16_t, uint16_t>>& registerValues)
{
    for (const auto& addrValuePair : registerValues)
        mRegistersMap->SetDefaultValue(addrValuePair.first, addrValuePair.second);
}

bool LMS7002M::IsSynced()
{
    if (!controlPort)
        return false;

    ChannelScope scope(this);

    std::vector<uint16_t> addrToRead = mRegistersMap->GetUsedAddresses(0);
    std::vector<uint16_t> dataReceived;
    dataReceived.resize(addrToRead.size(), 0);

    this->SetActiveChannel(Channel::ChA);
    std::vector<uint32_t> dataWr(addrToRead.size());
    std::vector<uint32_t> dataRd(addrToRead.size());
    for (size_t i = 0; i < addrToRead.size(); ++i)
        dataWr[i] = (static_cast<uint32_t>(addrToRead[i]) << 16);
    controlPort->SPI(dataWr.data(), dataRd.data(), dataWr.size());

    for (size_t i = 0; i < addrToRead.size(); ++i)
        dataReceived[i] = dataRd[i] & 0xFFFF;

    //check if local copy matches chip
    for (uint16_t i = 0; i < addrToRead.size(); ++i)
    {
        uint16_t regValue = mRegistersMap->GetValue(0, addrToRead[i]);
        if (addrToRead[i] <= readOnlyRegisters[readOnlyRegisters.size() - 1].address &&
            addrToRead[i] >= readOnlyRegisters[0].address)
        {
            //mask out readonly bits
            for (std::size_t j = 0; j < readOnlyRegisters.size(); ++j)
            {
                if (readOnlyRegisters[j].address == addrToRead[i])
                {
                    dataReceived[i] &= readOnlyRegisters[j].mask;
                    regValue &= readOnlyRegisters[j].mask;
                    break;
                }
            }
        }
        if (dataReceived[i] != regValue)
        {
            lime::debug("Addr: 0x%04X  gui: 0x%04X  chip: 0x%04X", addrToRead[i], regValue, dataReceived[i]);
            return false;
        }
    }

    addrToRead.clear(); //add only B channel addresses
    addrToRead = mRegistersMap->GetUsedAddresses(1);
    dataWr.resize(addrToRead.size());
    dataRd.resize(addrToRead.size());
    for (size_t i = 0; i < addrToRead.size(); ++i)
        dataWr[i] = (static_cast<uint32_t>(addrToRead[i]) << 16);
    controlPort->SPI(dataWr.data(), dataRd.data(), dataWr.size());
    for (size_t i = 0; i < addrToRead.size(); ++i)
        dataReceived[i] = dataRd[i] & 0xFFFF;
    this->SetActiveChannel(Channel::ChB);

    //check if local copy matches chip
    for (uint16_t i = 0; i < addrToRead.size(); ++i)
    {
        uint16_t regValue = mRegistersMap->GetValue(1, addrToRead[i]);
        if (addrToRead[i] <= readOnlyRegisters[readOnlyRegisters.size() - 1].address &&
            addrToRead[i] >= readOnlyRegisters[0].address)
        {
            //mask out readonly bits
            for (std::size_t j = 0; j < readOnlyRegisters.size(); ++j)
            {
                if (readOnlyRegisters[j].address == addrToRead[i])
                {
                    dataReceived[i] &= readOnlyRegisters[j].mask;
                    regValue &= readOnlyRegisters[j].mask;
                    break;
                }
            }
        }
        if (dataReceived[i] != regValue)
        {
            lime::debug("Addr: 0x%04X  gui: 0x%04X  chip: 0x%04X", addrToRead[i], regValue, dataReceived[i]);
            return false;
        }
    }

    return true;
}

OpStatus LMS7002M::UploadAll()
{
    if (!controlPort)
        return ReportError(OpStatus::IOFailure, "No device connected"s);

    ChannelScope scope(this);

    OpStatus status;

    std::vector<uint16_t> addrToWrite;
    std::vector<uint16_t> dataToWrite;

    uint16_t x0020_value = mRegistersMap->GetValue(0, 0x0020);
    this->SetActiveChannel(Channel::ChA); //select A channel

    addrToWrite = mRegistersMap->GetUsedAddresses(0);
    //remove 0x0020 register from list, to not change MAC
    addrToWrite.erase(std::find(addrToWrite.begin(), addrToWrite.end(), 0x0020));
    for (auto address : addrToWrite)
        dataToWrite.push_back(mRegistersMap->GetValue(0, address));

    status = SPI_write_batch(&addrToWrite[0], &dataToWrite[0], addrToWrite.size(), true);
    if (status != OpStatus::Success)
        return status;
    //after all channel A registers have been written, update 0x0020 register value
    status = SPI_write(0x0020, x0020_value);
    if (status != OpStatus::Success)
        return status;
    this->SetActiveChannel(Channel::ChB);
    if (status != OpStatus::Success)
        return status;

    addrToWrite = mRegistersMap->GetUsedAddresses(1);
    dataToWrite.clear();
    for (auto address : addrToWrite)
    {
        dataToWrite.push_back(mRegistersMap->GetValue(1, address));
    }
    this->SetActiveChannel(Channel::ChB); //select B channel
    status = SPI_write_batch(&addrToWrite[0], &dataToWrite[0], addrToWrite.size(), true);
    if (status != OpStatus::Success)
        return status;

    return OpStatus::Success;
}

OpStatus LMS7002M::DownloadAll()
{
    if (!controlPort)
        return ReportError(OpStatus::IOFailure, "No device connected"s);

    OpStatus status;
    ChannelScope scope(this, true);

    std::vector<uint16_t> addrToRead = mRegistersMap->GetUsedAddresses(0);
    std::vector<uint16_t> dataReceived;
    dataReceived.resize(addrToRead.size(), 0);
    this->SetActiveChannel(Channel::ChA);
    status = SPI_read_batch(&addrToRead[0], &dataReceived[0], addrToRead.size());
    if (status != OpStatus::Success)
        return status;

    for (uint16_t i = 0; i < addrToRead.size(); ++i)
    {
        mRegistersMap->SetValue(0, addrToRead[i], dataReceived[i]);
    }

    addrToRead.clear(); //add only B channel addresses
    addrToRead = mRegistersMap->GetUsedAddresses(1);
    dataReceived.resize(addrToRead.size(), 0);

    this->SetActiveChannel(Channel::ChB);
    status = SPI_read_batch(&addrToRead[0], &dataReceived[0], addrToRead.size());
    if (status != OpStatus::Success)
        return status;
    for (uint16_t i = 0; i < addrToRead.size(); ++i)
        mRegistersMap->SetValue(1, addrToRead[i], dataReceived[i]);

    return OpStatus::Success;
}

OpStatus LMS7002M::SetInterfaceFrequency(float_type cgen_freq_Hz, const uint8_t hbi, const uint8_t hbd)
{
    lime_Result result = lms7002m_set_interface_frequency(mC_impl, cgen_freq_Hz, hbi, hbd);
    return ResultToStatus(result);
}

OpStatus LMS7002M::EnableSXTDD(bool tdd)
{
    lime_Result result = lms7002m_enable_sxtdd(mC_impl, tdd);
    return ResultToStatus(result);
}

OpStatus LMS7002M::SetDCOffset(TRXDir dir, const float_type I, const float_type Q)
{
    lime_Result result = lms7002m_set_dc_offset(mC_impl, dir == TRXDir::Tx, I, Q);
    return ResultToStatus(result);
}

OpStatus LMS7002M::GetDCOffset(TRXDir dir, float_type& I, float_type& Q)
{
    lime_Result result = lms7002m_get_dc_offset(mC_impl, dir == TRXDir::Tx, &I, &Q);
    return ResultToStatus(result);
}

OpStatus LMS7002M::SetIQBalance(const TRXDir dir, const float_type phase, const float_type gainI, const float_type gainQ)
{
    lime_Result result = lms7002m_set_i_q_balance(mC_impl, dir == TRXDir::Tx, phase, gainI, gainQ);
    return ResultToStatus(result);
}

OpStatus LMS7002M::GetIQBalance(const TRXDir dir, float_type& phase, float_type& gainI, float_type& gainQ)
{
    lime_Result result = lms7002m_get_i_q_balance(mC_impl, dir == TRXDir::Tx, &phase, &gainI, &gainQ);
    return ResultToStatus(result);
}

void LMS7002M::EnableValuesCache(bool enabled)
{
    useCache = enabled;
}

MCU_BD* LMS7002M::GetMCUControls() const
{
    return mcuControl;
}

OpStatus LMS7002M::CalibrateInternalADC(int clkDiv)
{
    lime_Result result = lms7002m_calibrate_internal_adc(mC_impl, clkDiv);
    return ResultToStatus(result);
}

OpStatus LMS7002M::CalibrateRP_BIAS()
{
    lime_Result result = lms7002m_calibrate_rp_bias(mC_impl);
    return ResultToStatus(result);
}

float_type LMS7002M::GetTemperature()
{
    return lms7002m_get_temperature(mC_impl);
}

OpStatus LMS7002M::CopyChannelRegisters(const Channel src, const Channel dest, const bool copySX)
{
    ChannelScope scope(this);

    std::vector<uint16_t> addrToWrite;
    addrToWrite = mRegistersMap->GetUsedAddresses(1);
    if (!copySX)
    {
        const auto& SXMemoryAddresses = MemorySectionAddresses.at(MemorySection::SX);
        for (uint32_t address = SXMemoryAddresses.at(0); address <= SXMemoryAddresses.at(1); ++address)
            addrToWrite.erase(std::find(addrToWrite.begin(), addrToWrite.end(), address));
    }
    for (auto address : addrToWrite)
    {
        uint16_t data = mRegistersMap->GetValue(src == Channel::ChA ? 0 : 1, address);
        mRegistersMap->SetValue(dest == Channel::ChA ? 0 : 1, address, data);
    }
    if (controlPort)
        UploadAll();

    return OpStatus::Success;
}

OpStatus LMS7002M::CalibrateAnalogRSSI_DC_Offset()
{
    lime_Result result = lms7002m_calibrate_analog_rssi_dc_offset(mC_impl);
    return ResultToStatus(result);
}

double LMS7002M::GetClockFreq(ClockID clk_id)
{
    return lms7002m_get_clock_frequency(mC_impl, static_cast<lms7002m_clock_id>(clk_id));
}

OpStatus LMS7002M::SetClockFreq(ClockID clk_id, double freq)
{
    switch (clk_id)
    {
    case ClockID::CLK_REFERENCE:
        // TODO: recalculate CGEN,SXR/T
        break;
    case ClockID::CLK_CGEN:
        return SetFrequencyCGEN(freq, true, nullptr);
        break;
    case ClockID::CLK_SXR:
        return SetFrequencySX(TRXDir::Rx, freq, nullptr);
        break;
    case ClockID::CLK_SXT:
        return SetFrequencySX(TRXDir::Rx, freq, nullptr);
        break;
    case ClockID::CLK_RXTSP:
    case ClockID::CLK_TXTSP:
        return ReportError(OpStatus::InvalidValue, "RxTSP/TxTSP Clocks are read only"s);
    default:
        return ReportError(OpStatus::InvalidValue, "LMS7002M::SetClockFreq Unknown clock id"s);
    }
    return OpStatus::Success;
}

float_type LMS7002M::GetSampleRate(TRXDir dir, Channel ch)
{
    return lms7002m_get_sample_rate(mC_impl, dir == TRXDir::Tx, static_cast<lms7002m_channel>(ch));
}

float_type LMS7002M::GetSampleRate(TRXDir dir)
{
    return lms7002m_get_sample_rate(mC_impl, dir == TRXDir::Tx, lms7002m_get_active_channel(mC_impl));
}

OpStatus LMS7002M::SetGFIRFilter(TRXDir dir, Channel ch, bool enabled, double bandwidth)
{
    ChannelScope scope(this, ch);
    const bool bypassFIR = !enabled;
    if (dir == TRXDir::Tx)
    {
        Modify_SPI_Reg_bits(LMS7002MCSR::GFIR1_BYP_TXTSP, bypassFIR);
        Modify_SPI_Reg_bits(LMS7002MCSR::GFIR2_BYP_TXTSP, bypassFIR);
        Modify_SPI_Reg_bits(LMS7002MCSR::GFIR3_BYP_TXTSP, bypassFIR);
    }
    else
    {
        Modify_SPI_Reg_bits(LMS7002MCSR::GFIR1_BYP_RXTSP, bypassFIR);
        Modify_SPI_Reg_bits(LMS7002MCSR::GFIR2_BYP_RXTSP, bypassFIR);
        Modify_SPI_Reg_bits(LMS7002MCSR::GFIR3_BYP_RXTSP, bypassFIR);
        const bool sisoDDR = Get_SPI_Reg_bits(LML1_SISODDR);
        const bool clockIsNotInverted = !(enabled | sisoDDR);
        if (ch == LMS7002M::Channel::ChB)
        {
            Modify_SPI_Reg_bits(LMS7002MCSR::CDSN_RXBLML, clockIsNotInverted);
            Modify_SPI_Reg_bits(LMS7002MCSR::CDS_RXBLML, enabled ? 3 : 0);
        }
        else
        {
            Modify_SPI_Reg_bits(LMS7002MCSR::CDSN_RXALML, clockIsNotInverted);
            Modify_SPI_Reg_bits(LMS7002MCSR::CDS_RXALML, enabled ? 3 : 0);
        }
    }
    if (!enabled)
        return OpStatus::Success;

    if (bandwidth <= 0)
        return OpStatus::InvalidValue;

    double w, w2;
    int L;
    int div = 1;

    bandwidth /= 1e6;
    double interface_MHz;
    int ratio;
    if (dir == TRXDir::Tx)
    {
        ratio = Get_SPI_Reg_bits(LMS7002MCSR::HBI_OVR_TXTSP);
    }
    else
    {
        ratio = Get_SPI_Reg_bits(LMS7002MCSR::HBD_OVR_RXTSP);
    }

    interface_MHz = GetReferenceClk_TSP(dir) / 1e6;
    if (ratio != 7)
        div = (2 << (ratio));

    w = (bandwidth / 2) / (interface_MHz / div);
    L = div > 8 ? 8 : div;
    div -= 1;

    w2 = w * 1.1;
    if (w2 > 0.495)
    {
        w2 = w * 1.05;
        if (w2 > 0.495)
        {
            lime::error("GFIR LPF cannot be set to the requested bandwidth (%f)", bandwidth);
            return OpStatus::Error;
        }
    }

    double coef[120];
    double coef2[40];

    GenerateFilter(L * 15, w, w2, 1.0, 0, coef);
    GenerateFilter(L * 5, w, w2, 1.0, 0, coef2);

    if (dir == TRXDir::Tx)
    {
        Modify_SPI_Reg_bits(LMS7002MCSR::GFIR1_N_TXTSP, div);
        Modify_SPI_Reg_bits(LMS7002MCSR::GFIR2_N_TXTSP, div);
        Modify_SPI_Reg_bits(LMS7002MCSR::GFIR3_N_TXTSP, div);
    }
    else
    {
        Modify_SPI_Reg_bits(LMS7002MCSR::GFIR1_N_RXTSP, div);
        Modify_SPI_Reg_bits(LMS7002MCSR::GFIR2_N_RXTSP, div);
        Modify_SPI_Reg_bits(LMS7002MCSR::GFIR3_N_RXTSP, div);
    }

    OpStatus status;
    if ((status = SetGFIRCoefficients(dir, 0, coef2, L * 5)) != OpStatus::Success)
        return status;
    if ((status = SetGFIRCoefficients(dir, 1, coef2, L * 5)) != OpStatus::Success)
        return status;
    if ((status = SetGFIRCoefficients(dir, 2, coef, L * 15)) != OpStatus::Success)
        return status;

    std::stringstream ss;
    ss << "LMS "sv << ToString(dir) << " GFIR coefficients (BW: "sv << bandwidth << " MHz):\n"sv;
    ss << "GFIR1 = GFIR2:"sv;
    for (int i = 0; i < L * 5; ++i)
        ss << " " << coef2[i];
    ss << std::endl;
    ss << "GFIR3:"sv;
    for (int i = 0; i < L * 15; ++i)
        ss << " "sv << coef[i];
    ss << std::endl;
    lime::info(ss.str());

    return ResetLogicRegisters();
}

void LMS7002M::SetOnCGENChangeCallback(CGENChangeCallbackType callback, void* userData)
{
    mCallback_onCGENChange = callback;
    mCallback_onCGENChange_userData = userData;
}

OpStatus LMS7002M::SetRxLPF(double rfBandwidth_Hz)
{
    const int tiaGain = Get_SPI_Reg_bits(G_TIA_RFE);
    if (tiaGain < 1 || tiaGain > 3)
        return ReportError(OpStatus::InvalidValue, "RxLPF: Invalid G_TIA gain value");

    Modify_SPI_Reg_bits(PD_TIA_RFE, 0);
    Modify_SPI_Reg_bits(EN_G_RFE, 1);

    Modify_SPI_Reg_bits(ICT_TIAMAIN_RFE, 2);
    Modify_SPI_Reg_bits(ICT_TIAOUT_RFE, 2);

    Modify_SPI_Reg_bits(ICT_LPF_IN_RBB, 0x0C);
    Modify_SPI_Reg_bits(ICT_LPF_OUT_RBB, 0x0C);

    Modify_SPI_Reg_bits(ICT_PGA_OUT_RBB, 0x14);
    Modify_SPI_Reg_bits(ICT_PGA_IN_RBB, 0x14);
    const int pgaGain = Get_SPI_Reg_bits(G_PGA_RBB);
    if (pgaGain != 12)
    {
        lime::warning("RxLPF modifying G_PGA_RBB %i -> 12", pgaGain);
        Modify_SPI_Reg_bits(G_PGA_RBB, 12);
    }

    Modify_SPI_Reg_bits(RCC_CTL_PGA_RBB, 0x18);
    Modify_SPI_Reg_bits(C_CTL_PGA_RBB, 1);

    const double rxLpfMin = (tiaGain == 1) ? 4e6 : 1.5e6;
    const double rxLpfMax = 160e6;
    if (rfBandwidth_Hz != 0 && (rfBandwidth_Hz < rxLpfMin || rfBandwidth_Hz > rxLpfMax))
    {
        lime::warning(
            "Requested RxLPF(%g) is out of range [%g - %g]. Clamping to valid range.", rfBandwidth_Hz, rxLpfMin, rxLpfMax);
        rfBandwidth_Hz = std::clamp(rfBandwidth_Hz, rxLpfMin, rxLpfMax);
    }

    const double bandwidth_MHz = rfBandwidth_Hz / 1e6;

    int TIA_C;
    if (tiaGain == 1)
        TIA_C = 120 * 45 / (bandwidth_MHz / 2 / 1.5) - 15;
    else
        TIA_C = 120 * 14 / (bandwidth_MHz / 2 / 1.5) - 10;
    TIA_C = std::clamp(TIA_C, 0, 4095);

    int TIA_RCOMP = std::clamp(15 - TIA_C * 2 / 100, 0, 15);

    int TIA_CCOMP = (TIA_C / 100) + (tiaGain == 1 ? 1 : 0);
    TIA_CCOMP = std::clamp(TIA_CCOMP, 0, 15);

    int RX_L_C = 120 * 18 / (bandwidth_MHz / 2 / 0.75) - 103;
    RX_L_C = std::clamp(RX_L_C, 0, 2047);

    int RX_H_C = 120 * 50 / (bandwidth_MHz / 2 / 0.75) - 50;
    RX_H_C = std::clamp(RX_H_C, 0, 255);

    lime::debug("RxLPF(%g): TIA_C=%i, TIA_RCOMP=%i, TIA_CCOMP=%i, RX_L_C=%i, RX_H_C=%i\n",
        rfBandwidth_Hz,
        TIA_C,
        TIA_RCOMP,
        TIA_CCOMP,
        RX_L_C,
        RX_H_C);

    uint16_t cfb_tia_rfe = TIA_C;
    uint16_t rcomp_tia_rfe = TIA_RCOMP;
    uint16_t ccomp_tia_rfe = TIA_CCOMP;
    uint16_t input_ctl_pga_rbb = 4;
    uint16_t c_ctl_lpfl_rbb = RX_L_C;
    uint16_t c_ctl_lpfh_rbb = RX_H_C;
    uint16_t powerDowns = 0xD; // 0x0115[3:0]

    const double ifbw = bandwidth_MHz / 2 / 0.75;
    const uint16_t rcc_ctl_lpfh_rbb = std::clamp(ifbw / 10 - 2, 0.0, 7.0);
    uint16_t rcc_ctl_lpfl_rbb = 5;
    if (ifbw >= 20)
        rcc_ctl_lpfl_rbb = 5;
    else if (ifbw >= 15)
        rcc_ctl_lpfl_rbb = 4;
    else if (ifbw >= 10)
        rcc_ctl_lpfl_rbb = 3;
    else if (ifbw >= 5)
        rcc_ctl_lpfl_rbb = 2;
    else if (ifbw >= 3)
        rcc_ctl_lpfl_rbb = 1;
    else // (bw>=1.5)
        rcc_ctl_lpfl_rbb = 0;

    if (rfBandwidth_Hz <= 0) // LPF bypass
    {
        lime::info("RxLPF bypassed");
        powerDowns = 0xD;
        input_ctl_pga_rbb = 2;
    }
    else if (rfBandwidth_Hz < rxLpfMin)
    {
        lime::warning("RxLPF(%g) frequency too low. Clamping to %g MHz.", rfBandwidth_Hz, rxLpfMin / 1e6);
        if (tiaGain == 1)
        {
            cfb_tia_rfe = 4035;
            rcc_ctl_lpfl_rbb = 1;
            c_ctl_lpfl_rbb = 707;
        }
        else
        {
            cfb_tia_rfe = 3350;
            rcc_ctl_lpfl_rbb = 0;
            c_ctl_lpfl_rbb = 2047;
        }
        rcomp_tia_rfe = 0;
        ccomp_tia_rfe = 15;
        powerDowns = 0x9;
        input_ctl_pga_rbb = 0;
    }
    else if (rxLpfMin <= rfBandwidth_Hz && rfBandwidth_Hz <= 30e6)
    {
        powerDowns = 0x9;
        input_ctl_pga_rbb = 0;
    }
    else if (30e6 <= rfBandwidth_Hz && rfBandwidth_Hz <= rxLpfMax)
    {
        powerDowns = 0x5;
        input_ctl_pga_rbb = 1;
    }

    Modify_SPI_Reg_bits(CFB_TIA_RFE, cfb_tia_rfe);
    Modify_SPI_Reg_bits(RCOMP_TIA_RFE, rcomp_tia_rfe);
    Modify_SPI_Reg_bits(CCOMP_TIA_RFE, ccomp_tia_rfe);
    Modify_SPI_Reg_bits(0x0115, 3, 0, powerDowns);
    Modify_SPI_Reg_bits(INPUT_CTL_PGA_RBB, input_ctl_pga_rbb);
    Modify_SPI_Reg_bits(C_CTL_LPFL_RBB, c_ctl_lpfl_rbb);
    Modify_SPI_Reg_bits(C_CTL_LPFH_RBB, c_ctl_lpfh_rbb);
    Modify_SPI_Reg_bits(RCC_CTL_LPFL_RBB, rcc_ctl_lpfl_rbb);
    Modify_SPI_Reg_bits(RCC_CTL_LPFH_RBB, rcc_ctl_lpfh_rbb);

    return OpStatus::Success;
}

OpStatus LMS7002M::SetTxLPF(double rfBandwidth_Hz)
{
    const double txLpfLowRange[2] = { 5e6, 33e6 };
    const double txLpfHighRange[2] = { 56e6, 160e6 };

    // common setup
    Modify_SPI_Reg_bits(0x0106, 15, 0, 0x318C);
    Modify_SPI_Reg_bits(0x0107, 15, 0, 0x318C);
    Modify_SPI_Reg_bits(ICT_IAMP_FRP_TBB, 8);
    Modify_SPI_Reg_bits(ICT_IAMP_GG_FRP_TBB, 12);
    Modify_SPI_Reg_bits(CCAL_LPFLAD_TBB, 31);
    Modify_SPI_Reg_bits(RCAL_LPFS5_TBB, 255);
    Modify_SPI_Reg_bits(R5_LPF_BYP_TBB, 1);
    Modify_SPI_Reg_bits(BYPLADDER_TBB, 0);

    uint16_t powerDowns = 0x15; // addr 0x0105[4:0]

    if (rfBandwidth_Hz <= 0) // Bypass LPF
    {
        lime::info("TxLPF bypassed");
        powerDowns = 0x15;
        Modify_SPI_Reg_bits(0x0105, 4, 0, powerDowns);
        Modify_SPI_Reg_bits(BYPLADDER_TBB, 1);
        return Modify_SPI_Reg_bits(RCAL_LPFS5_TBB, 0);
    }
    else if (rfBandwidth_Hz < txLpfLowRange[0] || txLpfHighRange[1] < rfBandwidth_Hz)
    {
        lime::warning("Requested TxLPF(%g) bandwidth is out of range [%g - %g]. Clamping to valid value.",
            rfBandwidth_Hz,
            txLpfLowRange[0],
            txLpfHighRange[1]);
        rfBandwidth_Hz = std::clamp(rfBandwidth_Hz, txLpfLowRange[0], txLpfHighRange[1]);
    }

    const double rfbandwidth_MHz = rfBandwidth_Hz / 1e6;
    int rcal_lpflad = 0;
    int rcal_lpfh = 0;

    if (rfBandwidth_Hz < 5.3e6)
    {
        lime::warning("TxLPF(%g) setting bandwidth to %g.", rfBandwidth_Hz, txLpfLowRange[0]);
        rcal_lpflad = 0;
        powerDowns = 0x11;
    }
    else if (rfBandwidth_Hz <= txLpfLowRange[1]) // 5.3-33 MHz
    {
        const double LADlog = 20.0 * std::log10(rfbandwidth_MHz / (2.6 * 2));
        double LADterm1;
        {
            double t1 = 1.92163e-15;
            double t2 = std::sqrt(5.9304678933309e99 * std::pow(LADlog, 2) - 1.64373265875744e101 * LADlog + 1.17784161390406e102);
            LADterm1 = t1 * std::pow(t2 + 7.70095311849832e49 * LADlog - 1.0672267662616e51, 1.0 / 3.0);
        }

        double LADterm2;
        {
            double t1 = 6.50934553014677e18;
            double t2 = std::sqrt(5.9304678933309e99 * std::pow(LADlog, 2) - 1.64373265875744e101 * LADlog + 1.17784161390406e102);
            double t3 = t2 + 7.70095311849832e49 * LADlog - 1.0672267662616e51;
            LADterm2 = t1 / std::pow(t3, 1.0 / 3.0);
        }
        rcal_lpflad = std::clamp(196.916 + LADterm1 - LADterm2, 0.0, 255.0);
        powerDowns = 0x11;
    }
    else if (txLpfLowRange[1] <= rfBandwidth_Hz && rfBandwidth_Hz <= txLpfHighRange[0]) // 33-56 MHz gap
    {
        lime::warning("Requested TxLPF(%g) is in frequency gap [%g-%g], setting bandwidth to %g.",
            rfBandwidth_Hz,
            txLpfLowRange[1],
            txLpfHighRange[0],
            txLpfHighRange[0]);
        rcal_lpfh = 0;
        powerDowns = 0x07;
    }
    else if (rfBandwidth_Hz <= txLpfHighRange[1]) // <160MHz
    {
        const double Hlog = 20 * std::log10(rfbandwidth_MHz / (28 * 2));
        double Hterm1;
        {
            double t1 = 5.66735e-16;
            double t2 = std::sqrt(1.21443429517649e103 * std::pow(Hlog, 2) - 2.85279160551735e104 * Hlog + 1.72772373636442e105);
            double t3 = std::pow(t2 + 3.48487344845762e51 * Hlog - 4.09310646098208e052, 1.0 / 3.0);
            Hterm1 = t1 * t3;
        }
        double Hterm2;
        {
            double t1 = 2.12037432410767e019;
            double t2 = std::sqrt(1.21443429517649e103 * std::pow(Hlog, 2) - 2.85279160551735e104 * Hlog + 1.72772373636442e105);
            double t3 = std::pow(t2 + 3.48487344845762e51 * Hlog - 4.09310646098208e052, 1.0 / 3.0);
            Hterm2 = t1 / t3;
        }
        rcal_lpfh = std::clamp(197.429 + Hterm1 - Hterm2, 0.0, 255.0);
        powerDowns = 0x07;
    }

    lime::debug("TxLPF(%g): LAD=%i, H=%i\n", rfBandwidth_Hz, rcal_lpflad, rcal_lpfh);

    Modify_SPI_Reg_bits(RCAL_LPFLAD_TBB, rcal_lpflad);
    Modify_SPI_Reg_bits(RCAL_LPFH_TBB, rcal_lpfh);
    return Modify_SPI_Reg_bits(0x0105, 4, 0, powerDowns);
}
