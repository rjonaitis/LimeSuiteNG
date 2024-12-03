include(FeatureSummary)

option(BUILD_DRIVER_LIMEPCIE "Build limepcie linux kernel module" ON)
add_feature_info("BUILD_DRIVER_LIMEPCIE" BUILD_DRIVER_LIMEPCIE "Build limepcie driver")
if (BUILD_DRIVER_LIMEPCIE)
    add_subdirectory(limepcie)
endif()
