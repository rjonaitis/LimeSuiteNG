########################################################################
# GTest
########################################################################
include(CMakeDependentOption)
include(FeatureSummary)
include(FetchContent)
fetchcontent_declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG a7f443b80b105f940225332ed3c31f2790092f47 # latest main at the time of writing this
)

set(INSTALL_GMOCK OFF) # Don't install the GMock headers
set(INSTALL_GTEST OFF) # Don't install the GTest headers

if(MSVC)
    set(gtest_force_shared_crt
        ON
        CACHE BOOL "" FORCE)
endif()

set(BUILD_SHARED_LIBS_ORIG ${BUILD_SHARED_LIBS})
# building gtest as static, so it woudn't need to be installed and could be embedeed into limesuiteng library
set(BUILD_SHARED_LIBS OFF)

fetchcontent_makeavailable(googletest)
set(BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS_ORIG})

# software unit tests
add_executable(gtest-runner TestRunner.cpp)
if(CMAKE_BINARY_DIR)
    set_target_properties(gtest-runner PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
endif()
target_link_libraries(gtest-runner PRIVATE limesuiteng)

# When building shared libs, the tests have to be built into the library to be able to access non exported symbols
if(BUILD_SHARED_LIBS)
    set_property(TARGET gtest PROPERTY POSITION_INDEPENDENT_CODE TRUE)
    set_property(TARGET gmock PROPERTY POSITION_INDEPENDENT_CODE TRUE)
    set(TESTS_CONTAINER_TARGET limesuiteng)
    target_compile_definitions(${TESTS_CONTAINER_TARGET} PRIVATE GTEST_LINKED_AS_SHARED_LIBRARY=0)
else()
    set(TESTS_CONTAINER_TARGET gtest-runner)
endif()

target_include_directories(${TESTS_CONTAINER_TARGET} PRIVATE ${PROJECT_SOURCE_DIR})
target_link_libraries(${TESTS_CONTAINER_TARGET} PUBLIC $<BUILD_INTERFACE:GTest::gtest> $<BUILD_INTERFACE:GTest::gmock>)
target_sources(${TESTS_CONTAINER_TARGET} PRIVATE TestEntry.cpp)

target_sources(
    ${TESTS_CONTAINER_TARGET}
    PRIVATE comms/SPI_utilities_tests.cpp
            # parsers/CoefficientFileParserTest.cpp
            protocols/BufferInterleavingTest.cpp)

add_subdirectory(embedded/lms7002m)

########################################################################
# Tests requiring actual hardware to run on
add_executable(gtest-runner-hardware TestRunnerHardware.cpp)
target_include_directories(gtest-runner-hardware PRIVATE ${PROJECT_SOURCE_DIR})
target_sources(
    gtest-runner-hardware
    PRIVATE LimeSuite_wrapper/streaming.cpp
            LimeSuite_wrapper/device.cpp
            streaming/streaming.cpp
            streaming/RFStream_tests.cpp
            boards/LMS7002M_SDRDevice_Fixture.cpp)

if(NOT MSVC)
    set_source_files_properties(streaming/streaming.cpp PROPERTIES COMPILE_FLAGS -Wno-deprecated-declarations)
endif()

target_compile_definitions(gtest-runner-hardware PRIVATE GTEST_LINKED_AS_SHARED_LIBRARY=0)
target_link_libraries(
    gtest-runner-hardware
    PRIVATE limesuiteng limesuiteng-legacyapi taywee-args cli-shared
    PUBLIC $<BUILD_INTERFACE:GTest::gtest> $<BUILD_INTERFACE:GTest::gmock>)
if(CMAKE_BINARY_DIR)
    set_target_properties(gtest-runner-hardware PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
endif()

### Registers tests to be runnable using CTest
# include(CTest)
# include(GoogleTest)
# gtest_discover_tests(${TESTS_CONTAINER_TARGET})

# cmake_dependent_option(ENABLE_CODE_COVERAGE "Use gcov/lcov to generate a code coverage report" ON "ENABLE_UNIT_TESTS" OFF)
# if(CMAKE_BUILD_TYPE STREQUAL "Debug"
#    AND ENABLE_CODE_COVERAGE
#    AND (NOT MSVC))
#     include(CodeCoverage)
#     setup_target_for_coverage_lcov(
#         NAME ${LIME_TEST_SUITE_NAME}_coverage EXECUTABLE ${LIME_TEST_SUITE_NAME} EXCLUDE "/usr/*" "build/*" "external/*" "tests/*")
#     target_link_libraries(${LIME_TEST_SUITE_NAME} PUBLIC gcov)
# endif()
