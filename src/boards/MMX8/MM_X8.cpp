#include "MM_X8.h"

#include <fcntl.h>
#include <sstream>

#include "Logger.h"
#include "LitePCIe.h"
#include "limesuite/LMS7002M.h"
#include "lms7002m/LMS7002M_validation.h"
#include "FPGA_common.h"
#include "DSP/Equalizer.h"

#include "boards/LimeSDR_XTRX/LimeSDR_XTRX.h"
#include "limesuite/DeviceNode.h"

#include "math.h"

namespace lime {

static SDRDevice::CustomParameter cp_vctcxo_dac = { "VCTCXO DAC (volatile)", 0, 0, 65535, false };

// Do not perform any unnecessary configuring to device in constructor, so you
// could read back it's state for debugging purposes
LimeSDR_MMX8::LimeSDR_MMX8(std::vector<std::shared_ptr<IComms>>& spiLMS7002M,
    std::vector<std::shared_ptr<IComms>>& spiFPGA,
    std::vector<std::shared_ptr<LitePCIe>> trxStreams,
    ISPI* adfComms)
    : mTRXStreamPorts(trxStreams)
{
    mMainFPGAcomms = spiFPGA[8];
    SDRDevice::Descriptor& desc = mDeviceDescriptor;
    desc.name = GetDeviceName(LMS_DEV_LIMESDR_MMX8);

    // LMS64CProtocol::FirmwareInfo fw;
    // LMS64CProtocol::GetFirmwareInfo(controlPipe, fw);
    // LMS64CProtocol::FirmwareToDescriptor(fw, desc);

    // mFPGA = new lime::FPGA_X3(spiFPGA, SPI_LMS7002M_1);
    // mFPGA->SetConnection(&mFPGAcomms);
    // FPGA::GatewareInfo gw = mFPGA->GetGatewareInfo();
    // FPGA::GatewareToDescriptor(gw, desc);

    desc.socTree = std::make_shared<DeviceNode>("X8", "SDRDevice", this);

    mADF = new ADF4002();
    // TODO: readback board's reference clock
    mADF->Initialize(adfComms, 30.72e6);
    desc.socTree->children.push_back(std::make_shared<DeviceNode>("ADF4002", "ADF4002", mADF));

    mSubDevices.resize(8);
    desc.spiSlaveIds["FPGA"] = 0;
    desc.memoryDevices.push_back({ "FPGA FLASH", static_cast<uint32_t>(eMemoryDevice::FPGA_FLASH) });

    desc.customParameters.push_back(cp_vctcxo_dac);
    for (size_t i = 0; i < mSubDevices.size(); ++i)
    {
        mSubDevices[i] = new LimeSDR_XTRX(spiLMS7002M[i], spiFPGA[i], trxStreams[i]);
        const SDRDevice::Descriptor& d = mSubDevices[i]->GetDescriptor();

        for (const auto& soc : d.rfSOC)
        {
            RFSOCDescriptor temp = soc;
            char ctemp[512];
            sprintf(ctemp, "%s@%li", temp.name.c_str(), i + 1);
            temp.name = std::string(ctemp);
            desc.rfSOC.push_back(temp);
        }

        for (const auto& s : d.spiSlaveIds)
        {
            char ctemp[512];
            sprintf(ctemp, "%s@%li", s.first.c_str(), i + 1);
            desc.spiSlaveIds[ctemp] = (i + 1) << 8 | s.second;
            chipSelectToDevice[desc.spiSlaveIds[ctemp]] = mSubDevices[i];
        }

        for (const auto& s : d.memoryDevices)
        {
            char ctemp[512];
            sprintf(ctemp, "%s@%li", s.name.c_str(), i + 1);
            lime::SDRDevice::DataStorage* storage = new lime::SDRDevice::DataStorage();
            storage->name = s.name;
            storage->id = (static_cast<uint32_t>(i) + 1) << 8 | s.id;
            storage->map = s.map;
            desc.memoryDevices.push_back(*storage);
            //desc.memoryDevices.push_back({ ctemp, (static_cast<uint32_t>(i) + 1) << 8 | s.id });
            memorySelectToDevice[(i + 1) << 8 | s.id] = mSubDevices[i];
        }

        for (const auto& s : d.customParameters)
        {
            SDRDevice::CustomParameter p = s;
            p.id |= (i + 1) << 8;
            char ctemp[512];
            sprintf(ctemp, "%s@%li", s.name.c_str(), i + 1);
            p.name = ctemp;
            desc.customParameters.push_back(p);
            customParameterToDevice[p.id] = mSubDevices[i];
        }

        char ctemp[256];
        sprintf(ctemp, "%s#%li", d.socTree->name.c_str(), i + 1);
        d.socTree->name = std::string(ctemp);
        desc.socTree->children.push_back(d.socTree);
    }
}

LimeSDR_MMX8::~LimeSDR_MMX8()
{
    for (size_t i = 0; i < mSubDevices.size(); ++i)
        delete mSubDevices[i];
}

const SDRDevice::Descriptor& LimeSDR_MMX8::GetDescriptor()
{
    return mDeviceDescriptor;
}

void LimeSDR_MMX8::Configure(const SDRConfig& cfg, uint8_t socIndex)
{
    try
    {
        mSubDevices[socIndex]->Init();
        mSubDevices[socIndex]->Configure(cfg, 0);
    } //try
    catch (std::logic_error& e)
    {
        printf("LimeSDR_MMX8 config: %s\n", e.what());
        throw;
    } catch (std::runtime_error& e)
    {
        throw;
    }
}

int LimeSDR_MMX8::Init()
{
    for (size_t i = 0; i < mSubDevices.size(); ++i)
        mSubDevices[i]->Init();
    return 0;
}

void LimeSDR_MMX8::Reset()
{
    for (uint32_t i = 0; i < mSubDevices.size(); ++i)
        mSubDevices[i]->Reset();
}

void LimeSDR_MMX8::GetGPSLock(GPS_Lock* status)
{
    // TODO: implement
}

double LimeSDR_MMX8::GetSampleRate(uint8_t moduleIndex, TRXDir trx)
{
    return mSubDevices[moduleIndex]->GetSampleRate(0, trx);
}

double LimeSDR_MMX8::GetClockFreq(uint8_t clk_id, uint8_t channel)
{
    return mSubDevices[channel / 2]->GetClockFreq(clk_id, channel & 0x1);
}

void LimeSDR_MMX8::SetClockFreq(uint8_t clk_id, double freq, uint8_t channel)
{
    mSubDevices[channel / 2]->SetClockFreq(clk_id, freq, channel & 1);
}

void LimeSDR_MMX8::Synchronize(bool toChip)
{
    for (auto& d : mSubDevices)
        d->Synchronize(toChip);
}

void LimeSDR_MMX8::EnableCache(bool enable)
{
    for (auto& d : mSubDevices)
        d->EnableCache(enable);
}

int LimeSDR_MMX8::StreamSetup(const StreamConfig& config, uint8_t moduleIndex)
{
    return mSubDevices[moduleIndex]->StreamSetup(config, 0);
}

void LimeSDR_MMX8::StreamStart(uint8_t moduleIndex)
{
    mSubDevices[moduleIndex]->StreamStart(0);
}

void LimeSDR_MMX8::StreamStop(uint8_t moduleIndex)
{
    mSubDevices[moduleIndex]->StreamStop(0);
}

int LimeSDR_MMX8::StreamRx(uint8_t moduleIndex, lime::complex32f_t** dest, uint32_t count, StreamMeta* meta)
{
    return mSubDevices[moduleIndex]->StreamRx(0, dest, count, meta);
}

int LimeSDR_MMX8::StreamRx(uint8_t moduleIndex, lime::complex16_t** dest, uint32_t count, StreamMeta* meta)
{
    return mSubDevices[moduleIndex]->StreamRx(0, dest, count, meta);
}

int LimeSDR_MMX8::StreamTx(uint8_t moduleIndex, const lime::complex32f_t* const* samples, uint32_t count, const StreamMeta* meta)
{
    return mSubDevices[moduleIndex]->StreamTx(0, samples, count, meta);
}

int LimeSDR_MMX8::StreamTx(uint8_t moduleIndex, const lime::complex16_t* const* samples, uint32_t count, const StreamMeta* meta)
{
    return mSubDevices[moduleIndex]->StreamTx(0, samples, count, meta);
}

void LimeSDR_MMX8::StreamStatus(uint8_t moduleIndex, SDRDevice::StreamStats* rx, SDRDevice::StreamStats* tx)
{
    mSubDevices[moduleIndex]->StreamStatus(0, rx, tx);
}

int LimeSDR_MMX8::SPI(uint32_t chipSelect, const uint32_t* MOSI, uint32_t* MISO, uint32_t count)
{
    if (chipSelect == 0)
    {
        return mMainFPGAcomms->SPI(MOSI, MISO, count);
    }

    SDRDevice* dev = chipSelectToDevice.at(chipSelect);
    if (!dev)
    {
        throw std::logic_error("invalid SPI chip select");
    }

    uint32_t subSelect = chipSelect & 0xFF;
    return dev->SPI(subSelect, MOSI, MISO, count);
}

int LimeSDR_MMX8::I2CWrite(int address, const uint8_t* data, uint32_t length)
{
    return -1;
}

int LimeSDR_MMX8::I2CRead(int addres, uint8_t* dest, uint32_t length)
{
    return -1;
}

void LimeSDR_MMX8::SetMessageLogCallback(LogCallbackType callback)
{
    for (size_t i = 0; i < mSubDevices.size(); ++i)
        mSubDevices[i]->SetMessageLogCallback(callback);
}

void* LimeSDR_MMX8::GetInternalChip(uint32_t index)
{
    return mSubDevices[index]->GetInternalChip(0);
}

int LimeSDR_MMX8::CustomParameterWrite(const std::vector<CustomParameterIO>& parameters)
{
    int ret = 0;

    for (const CustomParameterIO& param : parameters)
    {
        int subModuleIndex = (param.id >> 8) - 1;
        int id = param.id & 0xFF;

        std::vector<CustomParameterIO> parameter{ { id, param.value, param.units } };

        if (subModuleIndex >= 0)
            ret |= mSubDevices[subModuleIndex]->CustomParameterWrite(parameter);
        else
            ret |= mMainFPGAcomms->CustomParameterWrite(parameter);
    }

    return ret;
}

int LimeSDR_MMX8::CustomParameterRead(std::vector<CustomParameterIO>& parameters)
{
    int ret = 0;

    for (CustomParameterIO& param : parameters)
    {
        int subModuleIndex = (param.id >> 8) - 1;
        int id = param.id & 0xFF;

        std::vector<CustomParameterIO> parameter{ { id, param.value, param.units } };

        if (subModuleIndex >= 0)
            ret |= mSubDevices[subModuleIndex]->CustomParameterRead(parameter);
        else
            ret |= mMainFPGAcomms->CustomParameterRead(parameter);

        param.value = parameter[0].value;
        param.units = parameter[0].units;
    }

    return ret;
}

bool LimeSDR_MMX8::UploadMemory(uint32_t id, const char* data, size_t length, UploadMemoryCallback callback)
{
    if (id == 0)
    {
        int progMode;
        LMS64CProtocol::ProgramWriteTarget target;
        target = LMS64CProtocol::ProgramWriteTarget::FPGA;
        progMode = 1;
        return mMainFPGAcomms->ProgramWrite(data, length, progMode, target, callback);
    }

    SDRDevice* dev = memorySelectToDevice.at(id);
    if (!dev)
    {
        throw std::logic_error("invalid id select");
        return false;
    }

    uint32_t subSelect = id & 0xFF;
    return dev->UploadMemory(subSelect, data, length, callback);
}

int LimeSDR_MMX8::MemoryWrite(uint32_t id, uint32_t address, const void* data, size_t length)
{
    if (id == 0)
        return mMainFPGAcomms->MemoryWrite(address, data, length);

    SDRDevice* dev = memorySelectToDevice.at(id);
    if (!dev)
    {
        throw std::logic_error("invalid id select");
        return false;
    }

    uint32_t subSelect = id & 0xFF;
    return dev->MemoryWrite(subSelect, address, data, length);
}

int LimeSDR_MMX8::MemoryRead(uint32_t id, uint32_t address, void* data, size_t length)
{
    if (id == 0)
        return mMainFPGAcomms->MemoryRead(address, data, length);

    SDRDevice* dev = memorySelectToDevice.at(id);
    if (!dev)
    {
        throw std::logic_error("invalid id select");
        return false;
    }

    uint32_t subSelect = id & 0xFF;
    return dev->MemoryRead(subSelect, address, data, length);
}

int LimeSDR_MMX8::UploadTxWaveform(const StreamConfig& config, uint8_t moduleIndex, const void** samples, uint32_t count)
{
    return mSubDevices[moduleIndex]->UploadTxWaveform(config, 0, samples, count);
}

} //namespace lime
