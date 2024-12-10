include(FeatureSummary)
include(CMakeDependentOption)

########################################################################
## public headers
########################################################################
file(
    GLOB LIMESUITENG_PUBLIC_HEADERS
    RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/limesuiteng/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/limesuiteng/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/lime/*.h")

option(INSTALL_HEADERS "Enable install limesuiteng headers" ON)
add_feature_info("INSTALL_HEADERS" INSTALL_HEADERS "Install limesuiteng headers")
if(INSTALL_HEADERS)
    install(
        FILES ${LIMESUITENG_PUBLIC_HEADERS}
        DESTINATION include/limesuiteng
        COMPONENT liblimesuiteng)
endif()

########################################################################
# CMake Project config files
########################################################################
if(INSTALL_HEADERS)
    include(CMakePackageConfigHelpers)
    write_basic_package_version_file(
        ${PROJECT_BINARY_DIR}/limesuitengConfigVersion.cmake
        VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}
        COMPATIBILITY SameMinorVersion)
    install(
        FILES ${PROJECT_BINARY_DIR}/limesuitengConfigVersion.cmake
        DESTINATION lib${LIB_SUFFIX}/cmake/${PROJECT_NAME}
        COMPONENT liblimesuiteng)

    install(
        EXPORT limesuitengTarget
        NAMESPACE limesuiteng::
        FILE limesuitengConfig.cmake
        DESTINATION lib${LIB_SUFFIX}/cmake/${PROJECT_NAME}
        COMPONENT liblimesuiteng)

endif(INSTALL_HEADERS)

########################################################################
## lime suite library
########################################################################
set(LIME_SUITE_SOURCES
    CommonFunctions.cpp
    OEMTesting.cpp
    logger/Logger.cpp
    logger/LoggerCString.cpp
    protocols/LMS64CProtocol.cpp
    protocols/LMSBoards.cpp
    FPGA/FPGA_common.cpp
    FPGA/WriteRegistersBatch.cpp
    threadHelper/threadHelper.cpp
    memory/MemoryPool.cpp
    API/LimePlugin.cpp
    include/limesuiteng/SDRDevice.cpp
    utilities/toString.cpp)

if(NOT MSVC)
    set_source_files_properties(include/limesuiteng/SDRDevice.cpp PROPERTIES COMPILE_FLAGS -Wno-deprecated-declarations)
endif()

set_source_files_properties(mcu_program/common_src/lms7002m_controls.c PROPERTIES LANGUAGE CXX)

set(LIME_SUITE_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR} include boards lms7002m external threadHelper memory)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/limesuiteng/VersionInfo.in.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/include/limesuiteng/VersionInfo.cpp
    @ONLY)
list(APPEND LIME_SUITE_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/include/limesuiteng/VersionInfo.cpp)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/SystemResources.in.cpp ${CMAKE_CURRENT_BINARY_DIR}/SystemResources.cpp @ONLY)
list(APPEND LIME_SUITE_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/SystemResources.cpp)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

add_library(limesuiteng ${LIME_SUITE_SOURCES})

target_precompile_headers(limesuiteng PRIVATE ${CPP_STL_CPH})

set_target_properties(
    limesuiteng
    PROPERTIES POSITION_INDEPENDENT_CODE TRUE
               SOVERSION ${LIME_SUITE_SOVER}
               VERSION ${LIME_SUITE_LIBVER}
               DEFINE_SYMBOL "LIME_DLL_EXPORTS")

if(BUILD_SHARED_LIBS)
    target_compile_definitions(limesuiteng PUBLIC "LIME_DLL")
endif()

include(CheckAtomic)
if(NOT HAVE_CXX_ATOMICS_WITHOUT_LIB OR NOT HAVE_CXX_ATOMICS64_WITHOUT_LIB)
    target_link_libraries(limesuiteng PUBLIC atomic)
endif()

find_package(Threads REQUIRED)
target_link_libraries(
    limesuiteng
    PUBLIC Threads::Threads
    PRIVATE $<BUILD_INTERFACE:kissfft> $<BUILD_INTERFACE:rang> $<BUILD_INTERFACE:cpp-feather-ini-parser>)

target_include_directories(
    limesuiteng
    PRIVATE ${LIME_SUITE_INCLUDES}
    INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
              $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
              $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/external>
              $<INSTALL_INTERFACE:include>)

if(CMAKE_BUILD_TYPE STREQUAL "Debug"
   AND ENABLE_CODE_COVERAGE
   AND (NOT MSVC))
    include(CodeCoverage)
    append_coverage_compiler_flags_to_target(limesuiteng)

    if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        target_link_options(limesuiteng PUBLIC --coverage)
    endif()
endif()

if(WIN32) # Windows don't have RPATH support, so need to place the DLLs into same directory as binaries
    set_target_properties(limesuiteng PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
endif()

install(
    TARGETS limesuiteng
    EXPORT limesuitengTarget
    LIBRARY DESTINATION lib${LIB_SUFFIX} # .so file
    ARCHIVE DESTINATION lib${LIB_SUFFIX} # .lib file
    RUNTIME DESTINATION bin # .dll file
            COMPONENT liblimesuiteng)

# comms has to be processed before boards, to set up cmake variables needed for board support
add_subdirectory(comms)
add_subdirectory(boards)
add_subdirectory(chips)
add_subdirectory(DSP)
add_subdirectory(examples)
add_subdirectory(streaming)

########################################################################
# Build pkg config file
########################################################################
if(INSTALL_HEADERS)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/limesuiteng.pc.in ${CMAKE_CURRENT_BINARY_DIR}/limesuiteng.pc @ONLY)

    install(
        FILES ${CMAKE_CURRENT_BINARY_DIR}/limesuiteng.pc
        DESTINATION lib${LIB_SUFFIX}/pkgconfig
        COMPONENT liblimesuiteng)
endif(INSTALL_HEADERS)

########################################################################
# LMS_API Wrapper lib
# wrapper functions split into separate library to avoid symbols collision
# if both limesuiteng and LimeSuite would be linked to application.
########################################################################
add_library(limesuiteng-legacyapi API/LMS_APIWrapper.cpp)
target_include_directories(
    limesuiteng-legacyapi PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(limesuiteng-legacyapi PRIVATE limesuiteng)
