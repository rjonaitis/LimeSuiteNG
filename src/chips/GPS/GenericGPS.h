#pragma once

#include "interface/GPS.h"
#include <string>
#include <fstream>

namespace lime {

class ISerialPort;

class GenericGPS : public GPS
{
  public:
    GenericGPS(const std::string& ttyPath);
    ~GenericGPS();

    OpStatus SetPPSConfig(GPS::PPSmode availability, std::chrono::milliseconds pulseWidth) override;
    OpStatus StandByMode(bool sleep) override;

  private:
    std::string ttyPath;
    int tty_fd;
};

} // namespace lime
