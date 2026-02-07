#pragma once

#include <chrono>
#include <limesuiteng/OpStatus.h>

namespace lime {

class GPS
{
  public:
    virtual ~GPS(){};

    enum class PPSmode { Disable = 0, AfterFirstFix = 1, Only3DFix, Only2D3DFix, Always };
    virtual OpStatus SetPPSConfig(PPSmode availability, std::chrono::milliseconds pulseWidth) = 0;
    virtual OpStatus StandByMode(bool sleep) = 0;
};

} // namespace lime