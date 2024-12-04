cmake_minimum_required(VERSION 3.5)

project(
    limepcie
    VERSION 0.1.7
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
        set(GITHASH "")
    endif(GITHASH)
endif(get_git_head_revision)

# configure the version.h header in-tree, as the DKMS shell scripts are not aware of cmake build directory, and they copy all sources into system directory
configure_file(version.h.in ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-src/version.h @ONLY)
configure_file(dkms.conf.in ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-src/dkms.conf @ONLY)

find_program(DKMS_EXECUTABLE NAMES dkms)
cmake_dependent_option(LIMEPCIE_DKMS "Use DKMS when installing the kernel module" ON "DKMS_EXECUTABLE" OFF)
add_feature_info("LIMEPCIE_DKMS" LIMEPCIE_DKMS "Use DKMS system to build/install limepcie module")

# sources must include header files with relative path, so they would be copied into system
add_kernel_module(
    NAME
    limepcie
    VERSION
    ${PROJECT_VERSION}
    GITHASH
    ${GITHASH}
    SOURCES
    limepcie.h
    limepcie_core.c
    limeuart.h
    limeuart.c
    litex.h
    boards.h
    bsp/config.h
    bsp/csr.h
    bsp/flags.h
    bsp/mem.h
    bsp/soc.h
    CONFIGURED_FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-src/version.h
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-src/dkms.conf)

option(LIMEPCIE_INSTALL "Install Linux LimePCIe kernel module" ON)
if(NOT LIMEPCIE_INSTALL)
    return()
endif()

option(LIMEPCIE_RELOAD_ON_INSTALL "After install reload kernel driver modules" ON)
install_kernel_module(NAME limepcie DKMS ${LIMEPCIE_DKMS} RELOAD_MODULE ${LIMEPCIE_RELOAD_ON_INSTALL})
