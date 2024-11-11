#include "common.h"

#include "limesuiteng/SDRDescriptor.h"
#include "limesuiteng/OpStatus.h"
#include "limesuiteng/VersionInfo.h"

#include <assert.h>
#include <cstring>
#include <sstream>
#include <ctime>
#include <string_view>
#include <filesystem>
#include "utilities/toString.h"
#include "args.hxx"

#include "OEMTesting.h"

using namespace std;
using namespace lime;
using namespace lime::cli;
using namespace std::literals::string_literals;
using namespace std::literals::string_view_literals;

static bool interactiveMode = false;
static void WaitForUserInput()
{
    std::cerr << "Press any key to continue" << std::endl;
    cin.ignore();
}

class PrintOEMTestReporter : public OEMTestReporter
{
  public:
    PrintOEMTestReporter(uint64_t serialNumber, const std::filesystem::path& reportFilename)
    {
        if (!reportFilename.empty())
            fileOutput.open(reportFilename, std::fstream::out | std::fstream::app);
    };
    ~PrintOEMTestReporter() override
    {
        if (fileOutput.is_open())
        {
            int fileSize = fileOutput.tellg();
            if (fileSize == 0) // if file empty add column headers
            {
                for (const auto& h : headers)
                    fileOutput << h << ",";
                fileOutput << std::endl;
            }
            for (const auto& v : values)
                fileOutput << v << ",";
            fileOutput << std::endl;
        }
        fileOutput.close();
    }
    void OnStart(OEMTestData& test, const std::string& testName = std::string()) override
    {
        std::cerr << Indent() << "=== " << test.name << " ===" << std::endl;
        ++indentLevel;
    }
    void OnStepUpdate(OEMTestData& test, const std::string& text = std::string()) override
    {
        std::cerr << Indent() << text << std::endl;
    }
    void OnSuccess(OEMTestData& test) override
    {
        --indentLevel;
        std::cerr << Indent() << "=== " << test.name << " - PASSED"
                  << " ===" << std::endl;
        if (interactiveMode)
            WaitForUserInput();
    }
    void OnFail(OEMTestData& test, const std::string& reasonText = std::string()) override
    {
        assert(!test.passed);
        --indentLevel;
        std::cerr << Indent() << "=== " << test.name << " - FAILED";

        if (!reasonText.empty())
            std::cerr << " (" << reasonText << ")";
        std::cerr << " ===" << std::endl;
        if (interactiveMode)
            WaitForUserInput();
    }
    void ReportColumn(const std::string& header, const std::string& value) override
    {
        headers.push_back(header);
        values.push_back(value);
    }

  private:
    std::string Indent()
    {
        std::stringstream ss;
        for (int i = 0; i < indentLevel; ++i)
            ss << "  ";
        return ss.str();
    }
    int indentLevel{ 0 };
    std::fstream fileOutput;
    std::vector<std::string> headers;
    std::vector<std::string> values;
};

int main(int argc, char** argv)
{
    // clang-format off
    args::ArgumentParser            parser("limeOEM - utility for device testing and custom device specific operations", "");
    args::HelpFlag                  help(parser, "help", "This help", {'h', "help"});
    args::ValueFlag<std::string>    deviceFlag(parser, "device", "Specifies which device to use", {'d', "device"}, "");
    args::ValueFlag<std::string>    logFlag(parser, "level", "Log verbosity levels: error, warning, info, verbose, debug", {'l', "log"}, "error");

    args::ValueFlag<std::string>    reportFileFlag(parser, "", "File to append test results", {'o', "output"}, "");
    args::ValueFlag<uint64_t>       serialNumberFlag(parser, "decimal", "One time programmable serial number to be written to device", {"write-serial-number"}, 0);
    args::Flag                      runTestsFlag(parser, "", "Run tests to check device functionality", {"test"});
    args::Flag                      showVersion(parser, "", "Print software version", {"version"});
    args::Flag                      interactive(parser, "", "Wait for user input after each test", {"interactive"});
    // clang-format on

    try
    {
        parser.ParseCLI(argc, argv);
    } catch (const args::Help&)
    {
        cout << parser;
        return EXIT_SUCCESS;
    } catch (const std::exception& e)
    {
        cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    if (argc == 1)
    {
        std::cout << parser;
        return EXIT_SUCCESS;
    }

    if (showVersion)
    {
        cerr << GetLibraryVersion() << endl;
        if (argc == 2)
            return EXIT_SUCCESS;
    }
    interactiveMode = interactive;

    logVerbosity = strToLogLevel(args::get(logFlag));
    const std::string devName = args::get(deviceFlag);
    const std::string reportFilename = args::get(reportFileFlag);

    SDRDevice* device = ConnectToFilteredOrDefaultDevice(devName);
    if (!device)
        return EXIT_FAILURE;

    device->SetMessageLogCallback(lime::cli::LogCallback);
    lime::registerLogHandler(lime::cli::LogCallback);
    OpStatus result = OpStatus::Success;

    if (serialNumberFlag)
    {
        uint64_t serialNumberArg = args::get(serialNumberFlag);
        OpStatus status = device->WriteSerialNumber(serialNumberArg);
        if (status != OpStatus::Success)
        {
            cerr << "Failed to write serial number." << endl;
            DeviceRegistry::freeDevice(device);
            return EXIT_FAILURE;
        }
    }

    if (runTestsFlag)
    {
        uint64_t serialNumber = device->GetDescriptor().serialNumber;
        cerr << "Board serial number: " << serialNumber << " (";
        stringstream ss;
        ss << std::hex << std::setw(2) << std::setfill('0');
        for (size_t i = 0; i < sizeof(serialNumber); ++i)
            ss << "0x" << ((serialNumber >> 8 * i) & 0xFF) << " ";
        cerr << ss.str() << ")" << endl;

        PrintOEMTestReporter reporter(serialNumber, reportFilename);
        reporter.ReportColumn("S/N", std::to_string(serialNumber));
        result = device->OEMTest(&reporter);

        if (result == OpStatus::Success)
            cerr << "OEM TEST PASSED" << endl;
        else
            cerr << "OEM TEST FAILED" << endl;
    }

    DeviceRegistry::freeDevice(device);
    return result != OpStatus::Success ? EXIT_FAILURE : EXIT_SUCCESS;
}
