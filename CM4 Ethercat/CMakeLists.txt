cmake_minimum_required(VERSION 3.16)
project(klipper_ethercat C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -O2")

# Find IgH EtherLab headers and library
# Typical install location: /usr/local/include/ecrt.h, /usr/local/lib/libethercat.so
find_path(ETHERLAB_INCLUDE_DIR ecrt.h
    PATHS /usr/local/include /usr/include
    DOC "IgH EtherLab header directory"
)

find_library(ETHERLAB_LIBRARY ethercat
    PATHS /usr/local/lib /usr/lib
    DOC "IgH EtherLab library"
)

if(NOT ETHERLAB_INCLUDE_DIR OR NOT ETHERLAB_LIBRARY)
    message(WARNING "IgH EtherLab not found. Building without EtherCAT support "
                    "(useful for testing shared_mem and other components).")
    set(ETHERLAB_FOUND FALSE)
else()
    set(ETHERLAB_FOUND TRUE)
    message(STATUS "IgH EtherLab found: ${ETHERLAB_INCLUDE_DIR}, ${ETHERLAB_LIBRARY}")
endif()

# Source files for the RT process
set(RT_SOURCES
    src/ethercat_rt.c
    src/shared_mem.c
    src/cia402.c
    src/trajectory_interp.c
    src/pid_controller.c
    src/autotune.c
)

# Build the ethercat_rt binary (only if EtherLab is found)
if(ETHERLAB_FOUND)
    add_executable(ethercat_rt ${RT_SOURCES})

    target_include_directories(ethercat_rt PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${ETHERLAB_INCLUDE_DIR}
    )

    target_link_libraries(ethercat_rt
        ${ETHERLAB_LIBRARY}
        rt          # POSIX shared memory (shm_open)
        pthread     # pthreads
        m           # math library
    )

    # Install the binary
    install(TARGETS ethercat_rt DESTINATION /usr/local/bin)
endif()

# Build shared_mem as a standalone library (for testing)
add_library(shared_mem STATIC src/shared_mem.c)
target_include_directories(shared_mem PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(shared_mem rt)

# Build PID controller as standalone (for testing)
add_library(pid_controller STATIC src/pid_controller.c)
target_include_directories(pid_controller PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(pid_controller m)

# Build trajectory interpolator as standalone (for testing)
add_library(trajectory_interp STATIC src/trajectory_interp.c)
target_include_directories(trajectory_interp PUBLIC ${CMAKE_SOURCE_DIR}/src)

# Build autotune as standalone (for testing)
add_library(autotune STATIC src/autotune.c)
target_include_directories(autotune PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(autotune m pid_controller)

# Build CiA 402 as standalone (for testing)
add_library(cia402 STATIC src/cia402.c)
target_include_directories(cia402 PUBLIC ${CMAKE_SOURCE_DIR}/src)

# Test targets
enable_testing()

# Test shared memory
add_executable(test_shared_mem tests/test_shared_mem.c)
target_link_libraries(test_shared_mem shared_mem)
add_test(NAME test_shared_mem COMMAND test_shared_mem)

# Test PID controller
add_executable(test_pid tests/test_pid.c)
target_link_libraries(test_pid pid_controller m)
add_test(NAME test_pid COMMAND test_pid)

# Test CiA 402
add_executable(test_cia402 tests/test_cia402.c)
target_link_libraries(test_cia402 cia402)
add_test(NAME test_cia402 COMMAND test_cia402)

# Test trajectory interpolator
add_executable(test_trajectory tests/test_trajectory.c)
target_link_libraries(test_trajectory trajectory_interp rt)
add_test(NAME test_trajectory COMMAND test_trajectory)
