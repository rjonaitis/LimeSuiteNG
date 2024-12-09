########################################################################
## Octave plugin integration
########################################################################
add_subdirectory(amarisoft-plugin)
# add_subdirectory(octave)

########################################################################
## freedesktop environment integration
########################################################################
add_subdirectory(Desktop)

########################################################################
## SoapySDR bindings
########################################################################
add_subdirectory(Soapy_limesuiteng)

########################################################################
## GNU Radio plugin
########################################################################
find_package(Gnuradio "3.10")
include(CMakeDependentOption)
cmake_dependent_option(
    ENABLE_GNURADIO_PREREQ "Build LimeSuiteNG plugin for GNU Radio" ON "GNURADIO_FOUND;ENABLE_LIBRARY;BUILD_SHARED_LIBS" OFF)

if(ENABLE_GNURADIO_PREREQ)
    add_subdirectory(gr-limesuiteng)
else()
    add_feature_info(GNURADIO OFF "GNU Radio 3.10+ Plug-in")
endif()

########################################################################
## HDSDR plugin
########################################################################
add_subdirectory(HDSDR)
