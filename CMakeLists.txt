cmake_minimum_required(VERSION 3.10)
project(imu LANGUAGES C)

# Set C standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Core source files
set(IMU_CORE_SOURCES
    src/imu_core.c
)

# Driver source files
set(IMU_DRIVER_SOURCES "")
foreach(_drv IN LISTS SROBOTIS_PERIPHERALS_IMU_ENABLED_DRIVERS)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/${_drv}.c")
        list(APPEND IMU_DRIVER_SOURCES "src/drivers/${_drv}.c")
    else()
        message(FATAL_ERROR "imu: enabled driver '${_drv}' not found at src/drivers/${_drv}.c")
    endif()
endforeach()

# All source files
set(IMU_SOURCES
    ${IMU_CORE_SOURCES}
    ${IMU_DRIVER_SOURCES}
)

# Create shared library (drivers auto-register via constructor)
add_library(imu SHARED ${IMU_SOURCES})

# Link math library for trigonometric functions
target_link_libraries(imu m)

# Set compiler flags
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(imu PRIVATE
        -Wall
        -Wextra
    )
endif()

# Allow copied binaries to find libimu.so both next to the executable and in ../lib.
set(_IMU_RUNTIME_RPATH "$ORIGIN;$ORIGIN/../lib")

function(_imu_set_runtime_rpath target)
    set_target_properties(${target} PROPERTIES
        BUILD_RPATH "${_IMU_RUNTIME_RPATH}"
        INSTALL_RPATH "${_IMU_RUNTIME_RPATH}"
    )
endfunction()

# Install rules
install(TARGETS imu
    EXPORT ${SROBOTIS_EXPORT_SET}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(FILES include/imu.h
    DESTINATION include
)

# Optional: Create test executables
option(BUILD_TESTS "Build test programs" ON)

if(BUILD_TESTS)
    list(FIND SROBOTIS_PERIPHERALS_IMU_ENABLED_DRIVERS "drv_uart_cmp10a" _imu_uart_idx)
    if(_imu_uart_idx GREATER -1)
        add_executable(test_imu_uart test/test_imu_uart.c)
        target_link_libraries(test_imu_uart imu)
        _imu_set_runtime_rpath(test_imu_uart)
        install(TARGETS test_imu_uart RUNTIME DESTINATION bin)
    endif()

    list(FIND SROBOTIS_PERIPHERALS_IMU_ENABLED_DRIVERS "drv_i2c_mxc4005" _imu_i2c_idx)
    if(_imu_i2c_idx GREATER -1)
        add_executable(test_imu_i2c test/test_imu_i2c.c)
        target_link_libraries(test_imu_i2c imu)
        _imu_set_runtime_rpath(test_imu_i2c)
        install(TARGETS test_imu_i2c RUNTIME DESTINATION bin)
    endif()

    list(FIND SROBOTIS_PERIPHERALS_IMU_ENABLED_DRIVERS "drv_spi_icm42670p" _imu_spi_idx)
    if(_imu_spi_idx GREATER -1)
        add_executable(test_imu_spi test/test_imu_spi.c)
        target_link_libraries(test_imu_spi imu)
        _imu_set_runtime_rpath(test_imu_spi)
        install(TARGETS test_imu_spi RUNTIME DESTINATION bin)
    endif()
endif()
