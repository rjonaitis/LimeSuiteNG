########################################################################
## public headers
########################################################################

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/limesuiteng/VersionInfo.in.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/limesuiteng/VersionInfo.cpp
    @ONLY)
target_sources(limesuiteng PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/limesuiteng/VersionInfo.cpp)

if(INSTALL_DEVELOPMENT)
    file(
    GLOB LIMESUITENG_PUBLIC_HEADERS
    RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/limesuiteng/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/limesuiteng/*.hpp")

    install(
        FILES ${LIMESUITENG_PUBLIC_HEADERS}
        DESTINATION include/limesuiteng
        COMPONENT dev)

    file(
    GLOB LEGACY_PUBLIC_HEADERS
    RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/lime/*.h")

    install(
        FILES ${LEGACY_PUBLIC_HEADERS}
        DESTINATION include/lime
        COMPONENT dev)
endif()