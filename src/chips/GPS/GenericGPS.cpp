#include "GenericGPS.h"

#include "limesuiteng/Logger.h"

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <cstring>

#include <sstream>

using namespace std::literals::string_literals;

static const std::string pmtk_endl = "\r\n"s;

namespace lime {

static constexpr int pmtk_checksum(const std::string_view msg)
{
    size_t start = msg.find_first_of('$');
    if (start != std::string_view::npos)
        ++start;
    else
        start = 0;

    size_t end = msg.find_first_of('*');
    if (end == std::string_view::npos)
        end = msg.size();

    uint8_t checksum = 0;
    for (uint32_t i = start; i < end; ++i)
        checksum ^= uint8_t(msg[i]);
    return checksum;
}

static_assert(pmtk_checksum("PMTK161,0") == 0x28);
static_assert(pmtk_checksum("$PMTK161,0*") == 0x28);

static std::string checksum_string(const std::string_view msg)
{
    std::string text;
    text.resize(3);
    snprintf(text.data(), text.size(), "%02X", pmtk_checksum(msg));
    return text;
}

GenericGPS::GenericGPS(const std::string& ttyPath)
    : ttyPath(ttyPath)
{
    int flags = O_RDWR | O_CLOEXEC;
    tty_fd = open(ttyPath.c_str(), flags);
    if (tty_fd <= 0)
        printf("Failed to open %s\n", ttyPath.c_str());

    struct termios tty;
    if (tcgetattr(tty_fd, &tty) != 0)
    {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl

    // Disable any special handling of received bytes
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    if (tcsetattr(tty_fd, TCSANOW, &tty) != 0)
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
}

GenericGPS::~GenericGPS()
{
    close(tty_fd);
}

OpStatus GenericGPS::SetPPSConfig(GPS::PPSmode availability, std::chrono::milliseconds pulseWidth)
{
    std::stringstream ss;
    ss << "$PMTK285," << static_cast<uint32_t>(availability) << ',' << pulseWidth.count() << '*';
    ss << checksum_string(ss.str()) << pmtk_endl;

    const std::string msg = ss.str();
    lime::debug(msg);
    int written = write(tty_fd, msg.data(), msg.size());
    if (written != msg.size())
        return OpStatus::Error;
    return OpStatus::Success;
}

OpStatus GenericGPS::StandByMode(bool sleep)
{
    std::stringstream ss;
    ss << "$PMTK161,0*"s;
    ss << checksum_string(ss.str()) << pmtk_endl;

    const std::string msg = ss.str();
    if (sleep)
        lime::debug(msg);

    int written = write(tty_fd, msg.data(), msg.size());
    if (written != msg.size())
        return OpStatus::Error;
    return OpStatus::Success;
}

} // namespace lime