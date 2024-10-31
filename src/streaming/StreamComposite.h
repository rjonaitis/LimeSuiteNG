#pragma once

#include <vector>
#include <chrono>
#include <memory>
#include "limesuiteng/config.h"
#include "limesuiteng/complex.h"
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/StreamConfig.h"
#include "limesuiteng/RFStream.h"

namespace lime {

struct StreamMeta;

/** @brief Structure for holding information about the aggregate stream. */
struct LIME_API StreamAggregate {
    SDRDevice* device; ///< The device the stream is coming from.
    std::vector<int32_t> channels; ///< The channels the device is streaming with.
    int32_t streamIndex; ///< The index of the stream.
};

/** @brief Class for managing streaming from multiple devices at the same time. */
class LIME_API StreamComposite : public RFStream
{
  public:
    StreamComposite() = delete;

    /// @brief Constructs the StreamComposite object.
    /// @param aggregate The list of streams to aggregate and present as single combined stream.
    /// The composite takes over ownership of the aggregate streams.
    StreamComposite(std::vector<std::unique_ptr<RFStream>>& aggregate);

    /// @copydoc RFStream::Setup()
    OpStatus Setup(const StreamConfig& config) override;
    const StreamConfig& GetConfig() const override;

    /// @copydoc RFStream::Start()
    OpStatus Start() override;

    /// @copydoc RFStream::StageStart()
    OpStatus StageStart() override;

    /// @copydoc RFStream::Stop()
    void Stop() override;

    /// @copydoc RFStream::Teardown()
    void Teardown() override;

    /// @copydoc RFStream::StreamRx()
    uint32_t StreamRx(lime::complex32f_t* const* samples,
        uint32_t count,
        StreamMeta* meta,
        std::chrono::microseconds timeout = RFStream::DEFAULT_TIMEOUT) override;
    /// @copydoc RFStream::StreamRx()
    uint32_t StreamRx(lime::complex16_t* const* samples,
        uint32_t count,
        StreamMeta* meta,
        std::chrono::microseconds timeout = RFStream::DEFAULT_TIMEOUT) override;
    /// @copydoc RFStream::StreamRx()
    uint32_t StreamRx(lime::complex12_t* const* samples,
        uint32_t count,
        StreamMeta* meta,
        std::chrono::microseconds timeout = RFStream::DEFAULT_TIMEOUT) override;

    /// @copydoc RFStream::StreamTx()
    uint32_t StreamTx(const lime::complex32f_t* const* samples,
        uint32_t count,
        const StreamMeta* meta,
        std::chrono::microseconds timeout = RFStream::DEFAULT_TIMEOUT) override;
    /// @copydoc RFStream::StreamTx()
    uint32_t StreamTx(const lime::complex16_t* const* samples,
        uint32_t count,
        const StreamMeta* meta,
        std::chrono::microseconds timeout = RFStream::DEFAULT_TIMEOUT) override;
    /// @copydoc RFStream::StreamTx()
    uint32_t StreamTx(const lime::complex12_t* const* samples,
        uint32_t count,
        const StreamMeta* meta,
        std::chrono::microseconds timeout = RFStream::DEFAULT_TIMEOUT) override;

    /// @copydoc RFStream::StreamStatus()
    void StreamStatus(StreamStats* rx, StreamStats* tx) override;

    uint64_t GetHardwareTimestamp() const override;

  private:
    template<class T> uint32_t StreamRx_T(T* const* samples, uint32_t count, StreamMeta* meta, std::chrono::microseconds timeout);
    template<class T>
    uint32_t StreamTx_T(const T* const* samples, uint32_t count, const StreamMeta* meta, std::chrono::microseconds timeout);

    std::vector<StreamConfig> SplitAggregateStreamSetup(const StreamConfig& cfg);
    std::vector<std::unique_ptr<RFStream>> mAggregate;
    StreamConfig mConfig;
    bool warnAboutMisalignment; // warn only once if channels get desynced
};

} // namespace lime
