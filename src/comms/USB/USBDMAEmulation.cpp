#include "USBDMAEmulation.h"

#include "comms/USB/IUSB.h"

namespace lime {

// Too many async requests adds overhead and makes transfers timing consistency worse
static constexpr int maxAsyncTransfers = 16;

USBDMAEmulation::USBDMAEmulation(std::shared_ptr<IUSB> port, uint8_t endpoint, DataTransferDirection dir)
    : port(port)
    , counters{}
    , lastRequestIndex(0)
    , endpoint(endpoint)
    , dir(dir)
    , continuous(false)
{
    mappings.resize(maxAsyncTransfers);
    for (auto& memoryBlock : mappings)
    {
        memoryBlock.size = 65536;
        memoryBlock.buffer = new uint8_t[memoryBlock.size];
    }

    for (int i = 0; i < maxAsyncTransfers; ++i)
    {
        AsyncXfer* async = new AsyncXfer();
        async->requestedSize = 0;
        async->id = i;
        async->xfer = port->AllocateAsyncContext();

        transfers.push(async);
    }
}

OpStatus USBDMAEmulation::Initialize()
{
    return OpStatus::Success;
}

USBDMAEmulation::~USBDMAEmulation()
{
    AbortAllTransfers();
    while (!transfers.empty())
    {
        AsyncXfer* async = transfers.front();
        port->FreeAsyncContext(async->xfer);
        transfers.pop();
        delete async;
    }
    for (auto& memoryBlock : mappings)
        delete[] memoryBlock.buffer;
}

void USBDMAEmulation::AbortAllTransfers()
{
    std::vector<AsyncXfer*> temp;

    std::unique_lock lck{ queuesMutex };
    temp.reserve(pendingXfers.size());
    while (!pendingXfers.empty())
    {
        AsyncXfer* async = pendingXfers.front();
        pendingXfers.pop();
        port->AbortXfer(async->xfer);
        temp.push_back(async);
    }
    for (auto& async : temp)
    {
        port->WaitForXfer(async->xfer, 1000);
        port->FinishDataXfer(async->xfer);
        transfers.push(async);
    }
    assert(pendingXfers.empty());
    assert(transfers.size() == maxAsyncTransfers);
}

std::vector<IDMA::Buffer> USBDMAEmulation::GetBuffers() const
{
    return mappings;
}

std::string USBDMAEmulation::GetName() const
{
    return "usb";
}

OpStatus USBDMAEmulation::Enable(bool enable)
{
    continuous = false;
    if (!enable)
    {
        AbortAllTransfers();
        return OpStatus::Success;
    }

    counters.transfersCompleted = 0;
    lastRequestIndex = 0;

    // for USB nothing is needed to be done to just enable DMA
    return OpStatus::Success;
}

OpStatus USBDMAEmulation::EnableContinuous(bool enable, uint32_t maxTransferSize, uint8_t irqPeriod)
{
    OpStatus status = Enable(enable);
    continuous = true;

    if (!enable)
        return status;

    if (maxTransferSize == 0)
        return OpStatus::InvalidValue;

    if (dir != DataTransferDirection::DeviceToHost)
        return OpStatus::Success;
    // For continuous transferring, preemptively request data to be transferred
    std::unique_lock lck{ queuesMutex };
    while (!transfers.empty())
    {
        AsyncXfer* async = transfers.front();
        async->requestedSize = maxTransferSize;
        transfers.pop();
        status = port->BeginDataXfer(async->xfer, mappings[lastRequestIndex].buffer, async->requestedSize, endpoint);
        lastRequestIndex = (lastRequestIndex + 1) % mappings.size();
        if (status != OpStatus::Success)
            return status;
        pendingXfers.push(async);
    }
    return status;
}

void USBDMAEmulation::UpdateProducerStates()
{
    std::unique_lock lck{ queuesMutex };
    while (!pendingXfers.empty())
    {
        AsyncXfer* async = pendingXfers.front();
        int timeout_ms = 0; // just checking if the transfer is complete, not waiting.
        OpStatus status = port->WaitForXfer(async->xfer, timeout_ms);
        if (status != OpStatus::Success)
            break;
        port->FinishDataXfer(async->xfer);
        pendingXfers.pop();
        transfers.push(async);
        ++counters.transfersCompleted;
        counters.transfersCompleted &= 0xFFFF;
    }
    if (!continuous)
        return;
}

USBDMAEmulation::State USBDMAEmulation::GetCounters()
{
    UpdateProducerStates();
    return counters;
}

OpStatus USBDMAEmulation::SubmitRequest(uint64_t index, uint32_t bytesCount, DataTransferDirection dir, bool irq)
{
    assert(bytesCount > 0);
    assert(index < mappings.size());

    int count = 1;
    std::unique_lock lck{ queuesMutex };
    count = std::min(size_t(count), transfers.size());
    if (!transfers.empty() && count > 0)
    {
        AsyncXfer* async = transfers.front();
        async->requestedSize = bytesCount;
        OpStatus status = port->BeginDataXfer(async->xfer, mappings[index].buffer, async->requestedSize, endpoint);
        lastRequestIndex = index; //(lastRequestIndex + 1) % mappings.size();
        if (status != OpStatus::Success)
            return OpStatus::Error;
        transfers.pop();
        pendingXfers.push(async);
        --count;
        return OpStatus::Success;
    }
    return OpStatus::Error;
}

OpStatus USBDMAEmulation::Wait()
{
    std::unique_lock lck{ queuesMutex };
    if (pendingXfers.empty())
        return OpStatus::Success;

    AsyncXfer* async = pendingXfers.front();
    return port->WaitForXfer(async->xfer, 1000);
}

void USBDMAEmulation::BufferOwnership(uint16_t index, DataTransferDirection dir)
{
    // USB works with buffer copying, no need to transfer ownership/flush caches.
    // do nothing.
}

} // namespace lime
