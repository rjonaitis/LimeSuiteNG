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
# find_package(Gnuradio "3.10") # Can't use find gnuradio here, as it copies python_compiler_helper.py to top project dir, which is later not found within gr plugin build directory
add_subdirectory(gr-limesuiteng)

########################################################################
## HDSDR plugin
########################################################################
add_subdirectory(HDSDR)
