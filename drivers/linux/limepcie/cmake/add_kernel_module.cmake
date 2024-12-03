# Setup target for local build and install of kernel module

function(add_kernel_module)
    set(oneValueArgs NAME VERSION GITHASH KERNEL_RELEASE ARCH)
    set(multiValueArgs SOURCES INCLUDES CONFIGURED_FILES)
    cmake_parse_arguments("KMOD" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(NOT KMOD_NAME)
        message(FATAL_ERROR "Expected kernel module name")
    endif()

    # Get architecture
    if(NOT KMOD_ARCH)
        execute_process(
            COMMAND uname -m
            OUTPUT_VARIABLE KMOD_ARCH
            OUTPUT_STRIP_TRAILING_WHITESPACE)
        # ARCH might be 'aarch64', but the linux lib directories might be named 'arm64'
        if(${KMOD_ARCH} STREQUAL "aarch64" AND NOT EXISTS ${KERNEL_SOURCE_DIR}/arch/${KMOD_ARCH})
            set(KMOD_ARCH "arm64")
        endif()
    endif()

    if(NOT KMOD_KERNEL_RELEASE)
        execute_process(
            COMMAND uname -r
            OUTPUT_VARIABLE KMOD_KERNEL_RELEASE
            OUTPUT_STRIP_TRAILING_WHITESPACE)
    endif()

    # where Kbuild file will be placed
    set(KBUILD_FILE_DIR "${CMAKE_CURRENT_BINARY_DIR}/${KMOD_NAME}-src")

    # Generate the Kbuild file through cmake.
    set(KBUILD_INCLUDE_DIR_FLAGS "")
    foreach(dir ${KMOD_INCLUDES})
        string(APPEND KBUILD_INCLUDE_DIR_FLAGS "-I${KBUILD_FILE_DIR}/${dir}")
    endforeach()

    set(MODULE_KOBJECT ${KBUILD_FILE_DIR}/${KMOD_NAME}.ko)
    set(KERNEL_SOURCE_DIR /lib/modules/${KMOD_KERNEL_RELEASE}/build)
    set(KBUILD_CMD
        $(MAKE)
        -C
        ${KERNEL_SOURCE_DIR}
        ARCH=${KMOD_ARCH}
        # Informs kbuild that an external module is being built.
        # The value given to "M" is the absolute path of the directory where the external module (kbuild file) is located.
        M=${KBUILD_FILE_DIR}
        modules)

    set(KBUILD_CLEAN_CMD $(MAKE) -C ${KERNEL_SOURCE_DIR} ARCH=${KMOD_ARCH} M=${KBUILD_FILE_DIR} clean)

    add_custom_command(
        OUTPUT ${MODULE_KOBJECT}
        COMMAND ${KBUILD_CLEAN_CMD}
        COMMAND ${KBUILD_CMD}
        WORKING_DIRECTORY ${KBUILD_FILE_DIR}
        DEPENDS ${CMAKE_CURRENT_LIST_DIR} # rebuild if anything changes in the source dir
        VERBATIM
        COMMENT "Building Linux kernel module (${KMOD_NAME}) in dir: ${KBUILD_FILE_DIR}")

    add_custom_target(${KMOD_NAME} ALL DEPENDS ${MODULE_KOBJECT})
    set_target_properties(${KMOD_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${KBUILD_FILE_DIR})
    if(KMOD_VERSION)
        set_target_properties(${KMOD_NAME} PROPERTIES VERSION ${KMOD_VERSION})
    endif()

    # add external configured files to sources,so they get copied during install
    foreach(SRC_FILENAME ${KMOD_CONFIGURED_FILES})
        target_sources(${KMOD_NAME} PRIVATE ${SRC_FILENAME})
    endforeach()

    # Copy all source files into build directory and compile there, as the Kbuild produces artifacts in tree
    foreach(SRC_FILENAME ${KMOD_SOURCES})
        configure_file(${SRC_FILENAME} ${KBUILD_FILE_DIR}/${SRC_FILENAME} COPYONLY)
    endforeach()
    target_sources(${KMOD_NAME} PRIVATE ${KMOD_SOURCES})

    # Pick compilable source files
    string(REGEX MATCHALL "[^ ;]+[.]c" KERNEL_OBJECTS_RELATIVE_PATHS "${KMOD_SOURCES}")
    # Collect output object files that should be linked into single module
    string(REGEX REPLACE "[.]c[;]?" ".o " KERNEL_OBJECTS_RELATIVE_PATHS "${KERNEL_OBJECTS_RELATIVE_PATHS}")

    # Kernel modules consisting of multiple source files cannot have name that matches o
    list(FIND ${KERNEL_OBJECTS_RELATIVE_PATHS} "${KMOD_NAME}.c" SRC_FILENAME_MATCHES)
    if(NOT ${SRC_FILENAME_MATCHES} EQUAL -1)
        message(FATAL_ERROR "Kernel module consisting of multiple files cannot have name that matches source filename")
    endif()

    # KBuild file to define flags and dependencies
    file(
        WRITE ${KBUILD_FILE_DIR}/Kbuild
        "ccflags-y += -Wno-declaration-after-statement
    ccflags-y += -std=gnu11
    ccflags-y += ${KBUILD_INCLUDE_DIR_FLAGS}
    obj-m = ${KMOD_NAME}.o
    ${KMOD_NAME}-y := ${KERNEL_OBJECTS_RELATIVE_PATHS}
")

    # Simple MakeFile to build and clean. Makefile requires tabs for indendation
    file(
        WRITE ${KBUILD_FILE_DIR}/Makefile
        "
KERNEL_DIR ?= /lib/modules/`uname -r`/build
default:
\tmake -C $(KERNEL_DIR) M=$$PWD modules
clean:
\tmake -C $(KERNEL_DIR) M=$$PWD clean
\trm -f *~
")
endfunction()

function(install_kernel_module_modprobe)
    set(oneValueArgs NAME)
    cmake_parse_arguments("KMOD_INSTALL" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(NOT KMOD_KERNEL_RELEASE)
        execute_process(
            COMMAND uname -r
            OUTPUT_VARIABLE KMOD_KERNEL_RELEASE
            OUTPUT_STRIP_TRAILING_WHITESPACE)
    endif()

    get_target_property(OBJECTS_DIR ${KMOD_INSTALL_NAME} LIBRARY_OUTPUT_DIRECTORY)
    install(FILES "${OBJECTS_DIR}/${KMOD_INSTALL_NAME}.ko" DESTINATION /lib/modules/${KMOD_KERNEL_RELEASE}/extra)

    # Remove previously active module
    install(CODE "execute_process(COMMAND modprobe -r ${KMOD_INSTALL_NAME})")
    # Reload new module
    install(CODE "execute_process(COMMAND modprobe ${KMOD_INSTALL_NAME})")
endfunction()

function(install_kernel_module_dkms)
    set(oneValueArgs NAME)
    cmake_parse_arguments("KMOD_INSTALL" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    get_target_property(OBJECTS_DIR ${KMOD_INSTALL_NAME} LIBRARY_OUTPUT_DIRECTORY)
    get_target_property(MODULE_VERSION ${KMOD_INSTALL_NAME} VERSION)

    # copy all source files into system directory, exclude local build objects
    install(
        DIRECTORY ${OBJECTS_DIR}/
        DESTINATION "/usr/src/${KMOD_INSTALL_NAME}-${MODULE_VERSION}"
        PATTERN "*.o" EXCLUDE
        PATTERN "*.cmd" EXCLUDE)

    install(CODE "execute_process(COMMAND sudo dkms add -m ${KMOD_INSTALL_NAME} -v ${MODULE_VERSION})")
    # unbuild to force rebuild if the same version's source has been updated
    install(CODE "execute_process(COMMAND sudo dkms unbuild -m ${KMOD_INSTALL_NAME} -v ${MODULE_VERSION})")
    # install also builds if it wasn't built yet
    install(CODE "execute_process(COMMAND sudo dkms install -m ${KMOD_INSTALL_NAME} -v ${MODULE_VERSION})")

    # dkms only installs, but does not load the driver, load it manually
    install(CODE "execute_process(COMMAND modprobe -r ${KMOD_INSTALL_NAME})")
    install(CODE "execute_process(COMMAND modprobe ${KMOD_INSTALL_NAME})")
endfunction()

function(install_kernel_module)
    set(oneValueArgs NAME DKMS)
    cmake_parse_arguments("KMOD_INSTALL" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(NOT KMOD_INSTALL_NAME)
        message(FATAL_ERROR "Expected kernel module name")
    endif()

    # verify if DKMS is available
    if(KMOD_INSTALL_DKMS)
        find_program(DKMS_EXECUTABLE NAMES dkms)
        if(NOT DKMS_EXECUTABLE)
            message(FATAL_ERROR "${KMOD_INSTALL_NAME} cannot install using dkms: dkms is not available")
        else()
            # 0 means success, so it's inverted
            execute_process(
                COMMAND ${DKMS_EXECUTABLE} --version
                RESULT_VARIABLE IS_DKMS_NOT_PRESENT
                OUTPUT_QUIET)
            if(IS_DKMS_NOT_PRESENT)
                message(FATAL_ERROR "${KMOD_INSTALL_NAME} cannot install using dkms: dkms is not available")
            endif()
        endif()
    endif()

    if(KMOD_INSTALL_DKMS)
        install_kernel_module_dkms(NAME ${KMOD_INSTALL_NAME})
    else()
        install_kernel_module_modprobe(NAME ${KMOD_INSTALL_NAME})
    endif()
endfunction()
