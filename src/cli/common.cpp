#include "common.h"

using namespace lime;

static bool FuzzyHandleMatch(const DeviceHandle& handle, const std::string& text)
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

static SDRDevice* ConnectUsingNameHint(const std::string& hintArguments)
{
    auto handles = DeviceRegistry::enumerate();
    if (handles.empty())
    {
        std::cerr << "No devices detected." << std::endl;
        return nullptr;
    }

    DeviceHandle deserializedHandle(hintArguments);
    std::vector<DeviceHandle> filteredHandles;
    for (const DeviceHandle& h : handles)
    {
        // compare hint as if it was in serialized handle form.
        // if it's not, compare using basic text search among handle fields
        if (h.IsEqualIgnoringEmpty(deserializedHandle) || FuzzyHandleMatch(h, hintArguments))
            filteredHandles.push_back(h);
    }

    if (filteredHandles.empty())
    {
        std::cerr << "No devices found that match: " << hintArguments << std::endl;
        return nullptr;
    }

    if (filteredHandles.size() > 1)
    {
        std::cerr << "Ambiguous device argument, matches:\n";
        for (const auto& h : handles)
            std::cerr << "\t" << h.Serialize() << std::endl;
        return nullptr;
    }

    SDRDevice* device = DeviceRegistry::makeDevice(filteredHandles.front());
    if (!device)
    {
        std::cerr << "Failed to connect to: " << filteredHandles.front().Serialize() << std::endl;
        return nullptr;
    }
    return device;
}

SDRDevice* ConnectToFilteredOrDefaultDevice(const std::string& argument)
{
    if (!argument.empty())
        return ConnectUsingNameHint(argument);

    // Connect to
    auto handles = DeviceRegistry::enumerate();
    if (handles.empty())
    {
        std::cerr << "No devices detected." << std::endl;
        return nullptr;
    }
    if (handles.size() > 1)
    {
        std::cerr << "Multiple devices detected, specify which one to use with -d, --device:" << std::endl;
        for (const DeviceHandle& h : handles)
            std::cerr << "\t" << h.Serialize() << std::endl;
        return nullptr;
    }
    else
    {
        SDRDevice* device = DeviceRegistry::makeDevice(handles.at(0));
        if (!device)
        {
            std::cerr << "Failed to connect to: " << handles.at(0).Serialize() << std::endl;
            return nullptr;
        }
        return device;
    }
}