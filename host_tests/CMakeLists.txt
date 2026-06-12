cmake_minimum_required(VERSION 3.16)
project(radio_robot_host CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-DHOST_BUILD=1)

# ---------------------------------------------------------------------------
# Root of the firmware repo: parent of this CMakeLists.txt's directory.
# CMAKE_SOURCE_DIR is the host_tests/ directory; repo root is one level up.
# ---------------------------------------------------------------------------
get_filename_component(REPO_ROOT "${CMAKE_SOURCE_DIR}/.." ABSOLUTE)

# ---------------------------------------------------------------------------
# Collect firmware sources, then exclude CODAL-dependent files.
# ---------------------------------------------------------------------------
file(GLOB APP_SOURCES     "${REPO_ROOT}/source/app/*.cpp")
file(GLOB CONTROL_SOURCES "${REPO_ROOT}/source/control/*.cpp")
file(GLOB ROBOT_SOURCES   "${REPO_ROOT}/source/robot/*.cpp")
file(GLOB MOCK_SOURCES    "${REPO_ROOT}/source/hal/mock/*.cpp")

# Exclude files that require CODAL types that can't be stubbed at file level.
list(FILTER APP_SOURCES     EXCLUDE REGEX ".*/DebugCommandable\\.cpp$")
list(FILTER APP_SOURCES     EXCLUDE REGEX ".*/WedgeTest\\.cpp$")
list(FILTER CONTROL_SOURCES EXCLUDE REGEX ".*/LoopScheduler\\.cpp$")

# NezhaHAL is a CODAL-only HAL — excluded; MockHAL is the host HAL.
# (NezhaHAL.cpp is in source/hal/, not source/hal/mock/, so it is not
# captured by MOCK_SOURCES — but filter anyway for safety.)

add_library(firmware_host SHARED
    ${APP_SOURCES}
    ${CONTROL_SOURCES}
    ${ROBOT_SOURCES}
    ${MOCK_SOURCES}
    # BenchOtosSensor lives in source/hal/ (not mock/) so it is not globbed
    # by MOCK_SOURCES.  Add it explicitly so the host build includes it.
    "${REPO_ROOT}/source/hal/BenchOtosSensor.cpp"
    "${CMAKE_SOURCE_DIR}/sim_api.cpp"
)

target_include_directories(firmware_host PRIVATE
    "${REPO_ROOT}/source"
    "${REPO_ROOT}/source/app"
    "${REPO_ROOT}/source/control"
    "${REPO_ROOT}/source/hal"
    "${REPO_ROOT}/source/hal/mock"
    "${REPO_ROOT}/source/robot"
    "${REPO_ROOT}/source/types"
)
