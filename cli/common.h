#ifndef LIMESUITENG_CLI_COMMON_H
#define LIMESUITENG_CLI_COMMON_H

#include <chrono>
#include <vector>
#include <cstdint>
#include <iostream>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <signal.h>
#include <string>
#include <string_view>
#include "args.hxx"

#include "limesuiteng/DeviceHandle.h"
#include "limesuiteng/DeviceRegistry.h"
#include "limesuiteng/Logger.h"
#include "limesuiteng/SDRDevice.h"

namespace lime::cli {

extern LogLevel logVerbosity;
LogLevel strToLogLevel(const std::string_view str);
void LogCallback(LogLevel lvl, const std::string& msg);
std::vector<int> ParseIntArray(args::NargsValueFlag<int>& flag);

bool FuzzyHandleMatch(const DeviceHandle& handle, const std::string_view text);
lime::SDRDevice* ConnectToFilteredOrDefaultDevice(const std::string_view argument);

int AntennaNameToIndex(const std::vector<std::string>& antennaNames, const std::string& name);

} // namespace lime::cli

#endif
