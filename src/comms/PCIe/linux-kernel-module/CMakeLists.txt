cmake_minimum_required(VERSION 3.5)

# Get the version number from header file
file(READ version.in.h VERSION_HEADER_CONTENT)
string(REGEX MATCH "LIMEPCIE_VERSION \"([0-9]+\\.[0-9]+\\.[0-9]+)\"" DRIVER_VERSION "${VERSION_HEADER_CONTENT}")

if(NOT DRIVER_VERSION)
    message(FATAL_ERROR "Failed to extract version number from version.h")
endif(NOT DRIVER_VERSION)
set(DRIVER_VERSION "${CMAKE_MATCH_1}")

project(
    limepcie-kernel
    VERSION ${DRIVER_VERSION}
    LANGUAGES C)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(add_kernel_module)

find_package(LinuxKernelHeaders REQUIRED)

# Get Git changeset
if(get_git_head_revision)
    get_git_head_revision(GITREFSPEC GITHASH)
    if(GITHASH)
        string(SUBSTRING "${GITHASH}" 0 8 GITHASH)
    else(GITHASH)
        set(GITHASH "123")
    endif(GITHASH)
endif(get_git_head_revision)

# configure the version.h header in-tree, as the DKMS shell scripts are not aware of cmake build directory, and they copy all sources into system directory
configure_file(${CMAKE_CURRENT_LIST_DIR}/version.in.h ${CMAKE_CURRENT_LIST_DIR}/version.h @ONLY)

cmake_dependent_option(INSTALL_KERNEL_MODULE "Install Linux LimePCIe kernel module" ON "LIMEPCIE_KERNEL_MODULE" ON)
if(NOT INSTALL_KERNEL_MODULE)
    return()
endif()

find_program(DKMS_EXECUTABLE NAMES dkms)
if(NOT DKMS_EXECUTABLE)
    set(IS_DKMS_NOT_PRESENT True)
else()
    # 0 means success, so it's inverted
    execute_process(
        COMMAND ${DKMS_EXECUTABLE} --version
        RESULT_VARIABLE IS_DKMS_NOT_PRESENT
        OUTPUT_QUIET)
endif()

cmake_dependent_option(USE_DKMS "Use DKMS when installing the kernel module" ON "NOT IS_DKMS_NOT_PRESENT" OFF)
add_feature_info(DKMS USE_DKMS "DKMS support for lime PCIe kernel module")

# If module is already loaded, unload it
install(CODE "execute_process(COMMAND ${CMAKE_CURRENT_LIST_DIR}/uninstall-legacy-module.sh)")

# sources must include header files with relative path, so they would be copied into system
add_kernel_module(
    NAME
    limepcie
    VERSION
    ${DRIVER_VERSION}
    GITHASH
    ${GITHASH}
    SOURCES
    version.h
    limepcie.c
    limepcie.h
    boards.h
    bsp/config.h
    bsp/csr.h
    bsp/flags.h
    bsp/mem.h
    bsp/soc.h
    INCLUDES
    bsp)
