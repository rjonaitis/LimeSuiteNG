if(WIN32)
    file(ARCHIVE_EXTRACT INPUT ${CMAKE_SOURCE_DIR}/external/FTD3XXLibrary.zip DESTINATION ${FETCHCONTENT_BASE_DIR}/)
    set(FTDI_FOUND TRUE)
    set(FTDI_INCLUDE_DIRS ${FETCHCONTENT_BASE_DIR}/FTD3XXLibrary)

    #library directory based on x64 vs x86
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(FTD3XX_LIB_DIR ${FTDI_INCLUDE_DIRS}/x64)
    else()
        set(FTD3XX_LIB_DIR ${FTDI_INCLUDE_DIRS}/Win32)
    endif()

    #default is to use static libs
    option(FTD3XX_STATIC "Use static libs for FTD3XX" TRUE)

    #when using static libs specify FTD3XX_STATIC
    if(FTD3XX_STATIC)
        set(FTDI_LIBRARIES ${FTD3XX_LIB_DIR}/Static_Lib/FTD3XX.lib SetupAPI)
        target_compile_definitions(limesuiteng PRIVATE FTD3XX_STATIC)
    else()
        #when using dynamic libs install the dll to bin/
        set(FTDI_LIBRARIES ${FTD3XX_LIB_DIR}/DLL/FTD3XX.lib SetupAPI)
        install(FILES ${FTD3XX_LIB_DIR}/DLL/FTD3XX.dll DESTINATION bin)
    endif()

    message(STATUS "FTDI_INCLUDE_DIRS: ${FTDI_INCLUDE_DIRS}")
    message(STATUS "FTDI_LIBRARIES: ${FTDI_LIBRARIES}")
    target_link_libraries(limesuiteng PRIVATE ${FTDI_LIBRARIES} legacy_stdio_definitions.lib)
    target_include_directories(limesuiteng PRIVATE ${FTDI_INCLUDE_DIRS})
endif()

include(FeatureSummary)
include(CMakeDependentOption)
cmake_dependent_option(LIMESUITENG_USB_FTDI "Enable FT601" ON "FTDI_FOUND OR LIBUSB_1_FOUND" OFF)
add_feature_info("LIMESUITENG_USB_FTDI" LIMESUITENG_USB_FTDI "USB support for FTDI")
if(NOT LIMESUITENG_USB_FTDI)
    return()
endif()

target_sources(limesuiteng PRIVATE FT601.cpp)
