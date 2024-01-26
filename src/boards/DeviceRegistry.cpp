#include "limesuite/DeviceRegistry.h"
#include "limesuite/SDRDevice.h"
#include "Logger.h"
#include <mutex>
#include <map>
#include <memory>
#include <iostream>
#include <iso646.h> // alternative operators for visual c++: not, and, or...
using namespace lime;

#if 0
    #define verbose_printf(...) lime::debug(__VA_ARGS__)
#else
    #define verbose_printf(...)
#endif

static std::mutex gRegistryMutex;
static std::map<std::string, DeviceRegistryEntry*> registryEntries;

void __loadBoardSupport();

/*******************************************************************
 * Registry implementation
 ******************************************************************/
std::vector<DeviceHandle> DeviceRegistry::enumerate(const DeviceHandle& hint)
{
    __loadBoardSupport();
    std::lock_guard<std::mutex> lock(gRegistryMutex);

    std::vector<DeviceHandle> results;
    for (const auto& entry : registryEntries)
    {
        // filter by media type if specified
        if (not hint.media.empty() and hint.media != entry.first)
            continue;

        for (auto handle : entry.second->enumerate(hint))
            results.push_back(handle);
    }
    return results;
}

SDRDevice* DeviceRegistry::makeDevice(const DeviceHandle& handle)
{
    __loadBoardSupport();
    std::lock_guard<std::mutex> lock(gRegistryMutex);

    //use the identifier as a hint to perform a discovery
    //only identifiers from the discovery function itself is used in the factory
    for (const auto& entry : registryEntries)
    {
        const auto r = entry.second->enumerate(handle);
        if (r.empty())
            continue;

        auto realHandle = r.front(); //just pick the first
        return entry.second->make(realHandle);
    }

    const std::string reason = "No devices found with given handle (" + handle.Serialize() + ")";
    throw(std::runtime_error(reason));

    return nullptr;
}

void DeviceRegistry::freeDevice(SDRDevice* device)
{
    //some client code may end up freeing a null connection
    if (device == nullptr)
        return;

    std::lock_guard<std::mutex> lock(gRegistryMutex);

    delete device;
}

std::vector<std::string> DeviceRegistry::moduleNames(void)
{
    __loadBoardSupport();
    std::vector<std::string> names;
    std::lock_guard<std::mutex> lock(gRegistryMutex);
    for (const auto& entry : registryEntries)
        names.push_back(entry.first);
    return names;
}

/*******************************************************************
 * Entry implementation
 ******************************************************************/
DeviceRegistryEntry::DeviceRegistryEntry(const std::string& name)
    : _name(name)
{
    std::lock_guard<std::mutex> lock(gRegistryMutex);
    registryEntries[_name] = this;
    verbose_printf("DeviceRegistry Added: %s\n", _name.c_str());
}

DeviceRegistryEntry::~DeviceRegistryEntry(void)
{
    std::lock_guard<std::mutex> lock(gRegistryMutex);
    registryEntries.erase(_name);
    verbose_printf("DeviceRegistry Removed: %s\n", _name.c_str());
}
