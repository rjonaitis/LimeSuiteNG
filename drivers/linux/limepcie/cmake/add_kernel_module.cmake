# Setup target for local build and install of kernel module

function(add_kernel_module)
    set(options USE_DKMS)
    set(oneValueArgs NAME VERSION GITHASH KERNEL_RELEASE ARCH)
    set(multiValueArgs SOURCES INCLUDES)
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

    # verify if DKMS is available
    if(KMOD_USE_DKMS)
        find_program(DKMS_EXECUTABLE NAMES dkms)
        if(NOT DKMS_EXECUTABLE)
            message(FATAL_ERROR "${KMOD_NAME} cannot build using dkms: dkms is not available")
        else()
            # 0 means success, so it's inverted
            execute_process(
                COMMAND ${DKMS_EXECUTABLE} --version
                RESULT_VARIABLE IS_DKMS_NOT_PRESENT
                OUTPUT_QUIET)
        endif()
    endif()

    if(NOT KMOD_KERNEL_RELEASE)
        execute_process(
            COMMAND uname -r
            OUTPUT_VARIABLE KMOD_KERNEL_RELEASE
            OUTPUT_STRIP_TRAILING_WHITESPACE)
    endif()

    # where Kbuild file will be placed
    set(KBUILD_FILE_DIR "${CMAKE_CURRENT_BINARY_DIR}/${KMOD_NAME}")

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

    set(MODE_FLAG "--dkms")
    if(NOT KMOD_USE_DKMS)
        set(KERNEL_MODULE_DESTINATION /lib/modules/${KMOD_KERNEL_RELEASE}/extra)
        install(FILES ${MODULE_KOBJECT} DESTINATION ${KERNEL_MODULE_DESTINATION})
        set(MODE_FLAG "--legacy")
    endif()

    # Copy all source files into build directory and compile there, as the Kbuild produces artifacts in tree
    set(OBJECT_FILES)
    foreach(SRC_FILENAME ${KMOD_SOURCES})
        configure_file(${CMAKE_CURRENT_LIST_DIR}/${SRC_FILENAME} ${KBUILD_FILE_DIR}/${SRC_FILENAME} COPYONLY)
    endforeach()

    # Collect output object files that should be linked into single module
    # message(WARNING "SRC FILES: " "${KMOD_SOURCES}")
    string(REGEX MATCHALL "[^ ;]+[.]c" KERNEL_OBJECTS_RELATIVE_PATHS "${KMOD_SOURCES}")
    # message(WARNING "C FILES: ${KERNEL_OBJECTS_RELATIVE_PATHS}")
    string(REGEX REPLACE "[.]c[;]?" ".o " KERNEL_OBJECTS_RELATIVE_PATHS "${KERNEL_OBJECTS_RELATIVE_PATHS}")
    # message(WARNING "OBJ FILES: ${KERNEL_OBJECTS_RELATIVE_PATHS}")

    # Kernel modules consisting of multiple source files cannot have name that matches o
    list(FIND ${KERNEL_OBJECTS_RELATIVE_PATHS} "${KMOD_NAME}.c" SRC_FILENAME_MATCHES)
    if (NOT ${SRC_FILENAME_MATCHES} EQUAL -1)
        message(FATAL_ERROR "Kernel module consisting of multiple files cannot have name that matches source filename")
    endif()

    file(
        WRITE ${KBUILD_FILE_DIR}/Kbuild
        "ccflags-y += -Wno-declaration-after-statement
    ccflags-y += -std=gnu11
    ccflags-y += ${KBUILD_INCLUDE_DIR_FLAGS}
    obj-m = ${KMOD_NAME}.o
    ${KMOD_NAME}-y := ${KERNEL_OBJECTS_RELATIVE_PATHS}
")

    # If module is already loaded, unload it
    install(CODE "execute_process(COMMAND ${CMAKE_CURRENT_LIST_DIR}/unload.sh ${KMOD_NAME})")

    install(
        CODE "execute_process(COMMAND ${CMAKE_CURRENT_LIST_DIR}/install.sh ${MODE_FLAG} ${KMOD_NAME} WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})"
    )

    add_custom_target(
        uninstall-${KMOD_NAME}
        COMMAND ${CMAKE_CURRENT_LIST_DIR}/uninstall.sh ${MODE_FLAG} ${KMOD_NAME}
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
        COMMENT "Uninstalling Linux kernel module ${KMOD_NAME}")

endfunction()
