#include "DeviceFactoryPCIe.h"

#include <fcntl.h>

#include "limesuiteng/DeviceHandle.h"
#include "CommonFunctions.h"
#include "limesuiteng/Logger.h"
#include "comms/PCIe/LimePCIe.h"
#include "protocols/LMSBoards.h"
#include "comms/PCIe/LMS64C_FPGA_Over_PCIe.h"
#include "comms/PCIe/LMS64C_LMS7002M_Over_PCIe.h"
#include "MMX8/LMS64C_ADF_Over_PCIe_MMX8.h"
#include "MMX8/LMS64C_FPGA_Over_PCIe_MMX8.h"
#include "MMX8/LMS64C_LMS7002M_Over_PCIe_MMX8.h"

#include "boards/LimeSDR_XTRX/LimeSDR_XTRX.h"
#include "boards/LimeSDR_X3/LimeSDR_X3.h"
#include "boards/MMX8/MM_X8.h"

#include <algorithm>

using namespace lime;
using namespace std::literals::string_literals;
using namespace std::literals::string_view_literals;

void __loadDeviceFactoryPCIe(void) //TODO fixme replace with LoadLibrary/dlopen
{
    static DeviceFactoryPCIe limePCIeSupport; // self register on initialization
}

DeviceFactoryPCIe::DeviceFactoryPCIe()
    : DeviceRegistryEntry("LimePCIe"s)
{
}

std::vector<DeviceHandle> DeviceFactoryPCIe::enumerate(const DeviceHandle& hint)
{
    std::vector<DeviceHandle> handles;
    DeviceHandle handle;
    handle.media = "PCIe"s;

    if (!hint.media.empty() && hint.media != handle.media)
        return handles;

    // generate handles by probing devices
    std::vector<std::string> nodes = LimePCIe::GetPCIeDeviceList();
    for (const std::string& nodeName : nodes)
    {
        // look for _control devices only, skip _trx*
        std::size_t nameEnd = nodeName.find("/control"sv);
        if (nameEnd == std::string::npos)
            continue;

        handle.addr = "/dev/"s + nodeName.substr(0, nameEnd); // removed _control postfix

        std::shared_ptr<LimePCIe> pcidev = std::make_shared<LimePCIe>();
        if (pcidev->Open(handle.addr + "/control0", O_RDWR) != OpStatus::Success)
            continue;

        // use GET_INFO command to recognize the device
        auto controlPipe = std::make_shared<PCIE_CSR_Pipe>(pcidev);
        LMS64CProtocol::FirmwareInfo fw{};
        int subDeviceIndex = 0;
        LMS64CProtocol::GetFirmwareInfo(*controlPipe, fw, subDeviceIndex);

        handle.name = GetDeviceName(static_cast<eLMS_DEV>(fw.deviceId));
        handle.serial = intToHex(fw.boardSerialNumber);

        // Add handle conditionally, filter by serial number
        if (handle.IsEqualIgnoringEmpty(hint))
            handles.push_back(handle);
    }
    return handles;
}

SDRDevice* DeviceFactoryPCIe::make(const DeviceHandle& handle)
{
    // Data transmission layer
    std::shared_ptr<LimePCIe> controlPort = std::make_shared<LimePCIe>();
    std::vector<std::shared_ptr<LimePCIe>> streamPorts;

    std::string controlFile(handle.addr + "/control0");
    OpStatus connectionStatus = controlPort->Open(controlFile, O_RDWR);
    if (connectionStatus != OpStatus::Success)
    {
        lime::ReportError(connectionStatus, "Unable to connect to device using handle (%s)", handle.Serialize().c_str());
        return nullptr;
    }

    std::vector<std::string> streamEndpoints = LimePCIe::GetEndpointsWithPattern(handle.addr, "trx*"s);
    std::sort(
        streamEndpoints.begin(), streamEndpoints.end()); // TODO: Fix potential sorting problem if there would be trx1 and trx11
    for (const std::string& endpointPath : streamEndpoints)
    {
        streamPorts.push_back(std::make_shared<LimePCIe>());
        streamPorts.back()->SetPathName(endpointPath);
    }

    // protocol layer
    auto route_lms7002m = std::make_shared<LMS64C_LMS7002M_Over_PCIe>(controlPort);
    auto route_fpga = std::make_shared<LMS64C_FPGA_Over_PCIe>(controlPort);

    LMS64CProtocol::FirmwareInfo fw{};
    int subDeviceIndex = 0;
    auto controlPipe = std::make_shared<PCIE_CSR_Pipe>(controlPort);
    LMS64CProtocol::GetFirmwareInfo(*controlPipe, fw, subDeviceIndex);

    switch (fw.deviceId)
    {
    case LMS_DEV_LIMESDR_XTRX:
        return new LimeSDR_XTRX(route_lms7002m, route_fpga, streamPorts.empty() ? nullptr : streamPorts.front(), controlPipe);
    case LMS_DEV_LIMESDR_X3:
        return new LimeSDR_X3(route_lms7002m, route_fpga, std::move(streamPorts), controlPipe);
    case LMS_DEV_LIMESDR_MMX8: {
        auto adfComms = std::make_shared<LMS64C_ADF_Over_PCIe_MMX8>(controlPort, 0);
        std::vector<std::shared_ptr<IComms>> controls(8);
        std::vector<std::shared_ptr<IComms>> fpga(8);

        for (size_t i = 0; i < controls.size(); ++i)
        {
            controls[i] = std::make_shared<LMS64C_LMS7002M_Over_PCIe_MMX8>(controlPort, i + 1);
            fpga[i] = std::make_shared<LMS64C_FPGA_Over_PCIe_MMX8>(controlPort, i + 1);
        }
        fpga.push_back(std::make_shared<LMS64C_FPGA_Over_PCIe_MMX8>(controlPort, 0));

        return new LimeSDR_MMX8(controls, fpga, std::move(streamPorts), controlPipe, adfComms);
    }
    default:
        lime::ReportError(OpStatus::InvalidValue, "Unrecognized device ID (%i)", fw.deviceId);
        return nullptr;
    }
}
