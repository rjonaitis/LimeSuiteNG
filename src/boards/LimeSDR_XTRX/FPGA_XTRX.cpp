#include "FPGA_XTRX.h"
#include "Logger.h"
#include <ciso646>
#include <vector>
#include <map>
#include <math.h>
#include <iostream>

#include "Register.h"

using namespace std::literals::string_literals;

namespace lime {

FPGA_XTRX::FPGA_XTRX(std::shared_ptr<ISPI> fpgaSPI, std::shared_ptr<ISPI> lms7002mSPI)
    : FPGA(fpgaSPI, lms7002mSPI)
{
}

OpStatus FPGA_XTRX::SetInterfaceFreq(double txRate_Hz, double rxRate_Hz, double txPhase, double rxPhase)
{
    lime::FPGA::FPGA_PLL_clock clocks[2];

    lime::debug("FPGA_XTRX"s);
    lime::info("Phases: tx phase %f rx phase %f", txPhase, rxPhase);

    clocks[0].index = 0;
    clocks[0].outFrequency = rxRate_Hz;
    clocks[1].index = 1;
    clocks[1].outFrequency = rxRate_Hz;
    clocks[1].phaseShift_deg = rxPhase;
    if (FPGA_XTRX::SetPllFrequency(1, rxRate_Hz, clocks, 2) != OpStatus::SUCCESS)
        return OpStatus::ERROR;

    clocks[0].index = 0;
    clocks[0].outFrequency = txRate_Hz;
    clocks[1].index = 1;
    clocks[1].outFrequency = txRate_Hz;
    clocks[1].phaseShift_deg = txPhase;
    if (FPGA_XTRX::SetPllFrequency(0, txRate_Hz, clocks, 2) != OpStatus::SUCCESS) //B.J.
        return OpStatus::ERROR;

    return OpStatus::SUCCESS;
}

OpStatus FPGA_XTRX::SetPllFrequency(const uint8_t pllIndex, const double inputFreq, FPGA_PLL_clock* clocks, const uint8_t clockCount)
{
    //Xilinx boards have different phase control mechanism
    double phase = clocks[1].phaseShift_deg;
    WriteRegister(0x0020, phase);
    return FPGA::SetPllFrequency(pllIndex, inputFreq, clocks, clockCount);
}

OpStatus FPGA_XTRX::SetInterfaceFreq(double txRate_Hz, double rxRate_Hz, int channel)
{
    if (channel == 1 || channel == 2)
        return OpStatus::SUCCESS;
    return FPGA::SetInterfaceFreq(txRate_Hz, rxRate_Hz, channel);
}

} //namespace lime
