#include "comms/PCIe/LimePCIeDMA.h"

#include <cassert>
#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

#include "comms/PCIe/LimePCIe.h"
#include "limesuiteng/Logger.h"
#ifdef __unix__
    #include <unistd.h>
    #include <fcntl.h>
    #include <poll.h>
    #include <sys/mman.h>
    #include <sys/ioctl.h>
    #include "drivers/linux/limepcie/limepcie.h"
#endif

using namespace std::literals::string_literals;

namespace lime {

LimePCIeDMA::LimePCIeDMA(std::shared_ptr<LimePCIe> port, DataTransferDirection dir)
    : port(port)
    , dir(dir)
    , isInitialized(false)
{
}

OpStatus LimePCIeDMA::Initialize()
{
    if (isInitialized)
        return OpStatus::Success;

    if (!port->IsOpen())
        port->Open(port->GetPathName(), O_RDWR | O_NOCTTY | O_CLOEXEC | O_NONBLOCK);
    limepcie_ioctl_mmap_dma_info info{};
    int ret = ioctl(port->mFileDescriptor, LIMEPCIE_IOCTL_MMAP_DMA_INFO, &info);
    if (ret != 0)
        return OpStatus::Error;

    mappings.reserve(info.dma_rx_buf_count);
    limepcie_ioctl_lock lockInfo{};

    if (dir == DataTransferDirection::DeviceToHost)
    {
        memset(&lockInfo, 0, sizeof(lockInfo));
        lockInfo.dma_writer_request = 1;
        ret = ioctl(port->mFileDescriptor, LIMEPCIE_IOCTL_LOCK, &lockInfo);
        if (ret != 0) //|| lockInfo.dma_writer_status == 0)
        {
            const std::string msg = ": DMA writer request denied"s;
            return OpStatus::PermissionDenied;
        }
        auto buf = static_cast<uint8_t*>(mmap(NULL,
            info.dma_rx_buf_size * info.dma_rx_buf_count,
            PROT_READ,
            MAP_SHARED,
            port->mFileDescriptor,
            info.dma_rx_buf_offset));
        if (buf == MAP_FAILED || buf == nullptr)
        {
            const std::string msg = ": failed to MMAP Rx DMA buffer"s;
            return OpStatus::Error;
        }
        for (size_t i = 0; i < info.dma_rx_buf_count; ++i)
            mappings.push_back({ buf + info.dma_rx_buf_size * i, info.dma_rx_buf_size });
    }
    else
    {
        memset(&lockInfo, 0, sizeof(lockInfo));
        lockInfo.dma_reader_request = 1;
        ret = ioctl(port->mFileDescriptor, LIMEPCIE_IOCTL_LOCK, &lockInfo);
        if (ret != 0) // || lockInfo.dma_reader_status == 0)
        {
            const std::string msg = ": DMA reader request denied"s;
            return OpStatus::PermissionDenied;
        }
        auto buf = static_cast<uint8_t*>(mmap(NULL,
            info.dma_tx_buf_size * info.dma_tx_buf_count,
            PROT_WRITE,
            MAP_SHARED,
            port->mFileDescriptor,
            info.dma_tx_buf_offset));
        if (buf == MAP_FAILED || buf == nullptr)
        {
            const std::string msg = ": failed to MMAP Tx DMA buffer"s;
            return OpStatus::Error;
        }
        for (size_t i = 0; i < info.dma_tx_buf_count; ++i)
            mappings.push_back({ buf + info.dma_tx_buf_size * i, info.dma_tx_buf_size });
    }
    isInitialized = true;
    return OpStatus::Success;
}

LimePCIeDMA::~LimePCIeDMA()
{
    if (!port->IsOpen() || !isInitialized)
        return;

    Enable(false);

    if (mappings.empty())
        return;

    munmap(mappings.front().buffer, mappings.front().size * mappings.size());

    limepcie_ioctl_lock lockInfo{ 0, 0, 0, 0, 0, 0 };
    if (dir == DataTransferDirection::DeviceToHost)
        lockInfo.dma_writer_release = 1;
    else
        lockInfo.dma_reader_release = 1;
    ioctl(port->mFileDescriptor, LIMEPCIE_IOCTL_LOCK, &lockInfo);
}

OpStatus LimePCIeDMA::Enable(bool enabled)
{
    if (!isInitialized)
        return OpStatus::Error;
    limepcie_ioctl_dma_control args{};
    args.enabled = enabled;
    args.directionFromDevice = (dir == DataTransferDirection::DeviceToHost);
    int ret = ioctl(port->mFileDescriptor, LIMEPCIE_IOCTL_DMA_CONTROL, &args);
    if (ret < 0)
        return ReportError(OpStatus::IOFailure, "Failed DMA Enable ioctl. errno(%i) %s", errno, strerror(errno));
    return OpStatus::Success;
}

OpStatus LimePCIeDMA::EnableContinuous(bool enabled, uint32_t maxTransferSize, uint8_t irqPeriod)
{
    if (!isInitialized)
        return OpStatus::Error;
    assert(port->IsOpen());

    limepcie_ioctl_dma_control_continuous args{};
    args.control.enabled = enabled;
    args.control.directionFromDevice = (dir == DataTransferDirection::DeviceToHost);
    args.transferSize = maxTransferSize;
    args.irqPeriod = irqPeriod;

    int ret = ioctl(port->mFileDescriptor, LIMEPCIE_IOCTL_DMA_CONTROL_CONTINUOUS, &args);
    if (ret < 0)
    {
        switch (ret)
        {
        case EBUSY:
            return ReportError(OpStatus::Busy, "DMA is already enabled. errno(%i) %s", errno, strerror(errno));
        default:
            return ReportError(OpStatus::IOFailure, "Failed DMA Enable continuous ioctl. errno(%i) %s", errno, strerror(errno));
        }
    }
    return OpStatus::Success;
}

IDMA::State LimePCIeDMA::GetCounters()
{
    IDMA::State dma{};
    if (!isInitialized)
        return dma;

    limepcie_ioctl_dma_status status{};
    status.wait_for_read = false;
    status.wait_for_write = false;
    int ret = ioctl(port->mFileDescriptor, LIMEPCIE_IOCTL_DMA_STATUS, &status);
    if (ret)
        throw std::runtime_error("TransmitLoop IOCTL failed to get DMA counters");
    dma.transfersCompleted = (dir == DataTransferDirection::DeviceToHost) ? status.fromDeviceCounter : status.toDeviceCounter;
    return dma;
}

OpStatus LimePCIeDMA::SubmitRequest(uint64_t index, uint32_t bytesCount, DataTransferDirection direction, bool generateIRQ)
{
    if (!isInitialized)
        return OpStatus::Error;
    assert(port->IsOpen());

    limepcie_ioctl_dma_request request{};
    request.bufferIndex = index;
    request.transferSize = bytesCount;
    request.generateIRQ = generateIRQ;
    request.directionFromDevice = (direction == DataTransferDirection::DeviceToHost);

    int ret = ioctl(port->mFileDescriptor, LIMEPCIE_IOCTL_DMA_REQUEST, &request);
    if (ret != 0)
        return OpStatus::Error;
    return OpStatus::Success;
}

OpStatus LimePCIeDMA::Wait()
{
    if (!isInitialized)
        return OpStatus::Error;
    assert(port->IsOpen());

    limepcie_ioctl_dma_status status{};
    status.wait_for_read = (dir == DataTransferDirection::DeviceToHost);
    status.wait_for_write = (dir == DataTransferDirection::HostToDevice);
    int ret = ioctl(port->mFileDescriptor, LIMEPCIE_IOCTL_DMA_STATUS, &status);
    if (ret)
        throw std::runtime_error("TransmitLoop IOCTL failed to get DMA counters");

    return OpStatus::Success;
}

void LimePCIeDMA::BufferOwnership(uint16_t index, DataTransferDirection bufferDirection)
{
    if (!isInitialized)
        return;

    limepcie_cache_flush args{};
    args.directionFromDevice = (dir == DataTransferDirection::DeviceToHost);
    args.sync_to_cpu = (bufferDirection == DataTransferDirection::DeviceToHost);
    args.bufferIndex = index;
    ioctl(port->mFileDescriptor, LIMEPCIE_IOCTL_CACHE_FLUSH, &args);
}

std::vector<IDMA::Buffer> LimePCIeDMA::GetBuffers() const
{
    return mappings;
}

std::string LimePCIeDMA::GetName() const
{
    return port->GetPathName();
}

} // namespace lime
