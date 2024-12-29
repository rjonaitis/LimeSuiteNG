project(
    limesuiteng-udev
    DESCRIPTION "Linux UDEV rules for LimeSuiteNG supported devices"
    VERSION 1.0.0.0
    LANGUAGES NONE
    HOMEPAGE_URL https://github.com/myriadrf/LimeSuiteNG/tree/develop/udev-rules)

set(UDEV_RULES_INSTALL_PATH
    "/etc/udev/rules.d"
    CACHE STRING "Path where to install udev rules")

install(
    FILES 65-limesuiteng-pcie.rules 65-limesuiteng-usb.rules
    DESTINATION ${UDEV_RULES_INSTALL_PATH}
    COMPONENT udev-rules)

option(UDEV_RULES_RELOAD "Reload UDEV rules after install" ON)
add_feature_info("UDEV_RULES_RELOAD" UDEV_RULES_RELOAD "Reload UDEV rules after install")
if(UDEV_RULES_RELOAD)
    install(
        CODE "
        MESSAGE(\"Reloading udev rules:\")
        execute_process(COMMAND udevadm control --reload-rules COMMAND_ECHO STDERR)
        execute_process(COMMAND udevadm trigger COMMAND_ECHO STDERR)"
        COMPONENT udev-rules)
endif()
