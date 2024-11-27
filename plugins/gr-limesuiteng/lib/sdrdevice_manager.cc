#include "sdrdevice_manager.h"

#include "limesuiteng/limesuiteng.hpp"

#include <memory>
#include <sstream>
#include <stdexcept>

using namespace lime;

namespace gr {
namespace limesuiteng {

static std::shared_ptr<sdrdevice_manager> g_deviceManager;

static bool FuzzyHandleMatch(const DeviceHandle& handle, const std::string_view text)
{
    if (text.empty())
        return true;

    if (!handle.name.empty() && handle.name.find(text) != std::string::npos)
        return true;

    if (!handle.addr.empty() && handle.addr.find(text) != std::string::npos)
        return true;

    if (!handle.serial.empty() && handle.serial.find(text) != std::string::npos)
        return true;

    if (!handle.media.empty() && handle.media.find(text) != std::string::npos)
        return true;

    return false;
}

std::shared_ptr<sdrdevice_manager> sdrdevice_manager::GetSingleton()
{
    if (!g_deviceManager)
        g_deviceManager = std::make_shared<sdrdevice_manager>();
    return g_deviceManager;
}

sdrdevice_manager::sdrdevice_manager() : _logger("SDRDeviceManager")
{
    _logger.debug("sdrdevice_manager created");
    enumeratedHandles = lime::DeviceRegistry::enumerate();
    if (enumeratedHandles.empty()) {
        throw std::runtime_error(
            "sdrdevice_manager::sdrdevice_manager(): No Lime devices found.");
    }

    // device->SetMessageLogCallback(lime::cli::LogCallback);
    // lime::registerLogHandler(lime::cli::LogCallback);
}

sdrdevice_manager::~sdrdevice_manager()
{
    _logger.debug("sdrdevice_manager destroyed");
    for (auto& ctx : m_contexts) {
        ctx->stream.reset();
        lime::DeviceRegistry::freeDevice(ctx->device.release());
    }
}

bool sdrdevice_manager::GetDeviceFullHandle(const std::string_view hintArguments,
                                            lime::DeviceHandle& handleOutput)
{
    if (enumeratedHandles.empty())
        return false;

    if (hintArguments.empty() && enumeratedHandles.size() == 1) {
        handleOutput = enumeratedHandles.front();
        return true;
    }

    DeviceHandle deserializedHandle(std::string{ hintArguments });
    std::vector<DeviceHandle> filteredHandles;
    for (const DeviceHandle& h : enumeratedHandles) {
        // compare hint as if it was in serialized handle form.
        // if it's not, compare using basic text search among handle fields
        if (h.IsEqualIgnoringEmpty(deserializedHandle) ||
            FuzzyHandleMatch(h, hintArguments))
            filteredHandles.push_back(h);
    }

    if (filteredHandles.empty()) {
        _logger.error("No devices match the handle");
        return false;
    }

    if (filteredHandles.size() > 1) {
        std::stringstream ss;
        ss << "Ambiguous device argument, matches:\n";
        for (const auto& h : filteredHandles)
            ss << '\t' << h.Serialize() << std::endl;
        _logger.error(ss.str());
        return false;
    }

    handleOutput = filteredHandles.front();
    return true;
}

std::shared_ptr<sdrdevice_context>
sdrdevice_manager::GetDeviceContextByHandle(const std::string& deviceHandleHint)
{
    DeviceHandle handle;
    if (!GetDeviceFullHandle(deviceHandleHint, handle))
        return nullptr;

    for (auto& ctx : m_contexts) {
        if (ctx->handle == handle)
            return ctx;
    }

    std::shared_ptr<sdrdevice_context>& ctx =
        m_contexts.emplace_back(std::make_shared<sdrdevice_context>());
    _logger.info(fmt::format("Connecting device: \"{:s}\"", handle.Serialize()));
    ctx->handle = handle;
    ctx->device = std::unique_ptr<SDRDevice>(DeviceRegistry::makeDevice(handle));
    if (!ctx->device) {
        m_contexts.pop_back();
        _logger.error(fmt::format("Unable to use device: \"{:s}\"", handle.Serialize()));
        return nullptr;
    }
    return ctx;
}

} // namespace limesuiteng
} // namespace gr
