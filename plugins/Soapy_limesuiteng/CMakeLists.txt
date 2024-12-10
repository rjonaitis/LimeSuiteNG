########################################################################
## SoapySDR bindings for LimeSuiteNG
########################################################################

find_package(SoapySDR CONFIG)
set_package_properties(
    SoapySDR PROPERTIES
    TYPE OPTIONAL
    PURPOSE "Adds LimeSuiteNG bindings for SoapySDR")

########################################################################
## Build support module
########################################################################
soapy_sdr_module_util(TARGET Soapy_limesuiteng SOURCES Registration.cpp Settings.cpp Streaming.cpp LIBRARIES limesuiteng)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # Disable warnings for unused parameters
    target_compile_options(Soapy_limesuiteng PRIVATE -Wno-unused-parameter)
endif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
