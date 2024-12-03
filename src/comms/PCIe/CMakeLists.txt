include(FeatureSummary)
include(CMakeDependentOption)
cmake_dependent_option(ENABLE_LIMEPCIE "Enable Lite PCIe" ON "UNIX; NOT APPLE" OFF)

if(UNIX)
    add_feature_info(LIMEPCIE ENABLE_LIMEPCIE "PCIe support")
endif()

if(NOT ENABLE_LIMEPCIE)
    return()
endif()

########################################################################
## Add to library
########################################################################
target_sources(
    limesuiteng PRIVATE LMS64C_FPGA_Over_PCIe.cpp LMS64C_LMS7002M_Over_PCIe.cpp LimePCIe.cpp LimePCIeDMA.cpp PCIE_CSR_Pipe.cpp)
