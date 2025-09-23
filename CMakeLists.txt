cmake_minimum_required(VERSION 3.22)

project(voxl-mavlink-mqtt-client)

# Standard compiler flags from other VOXL services
set(CMAKE_C_FLAGS "-std=gnu99 -O3 -Werror -Wall -Wextra -Wuninitialized \
	-Wunused-variable -Wdouble-promotion -Wmissing-prototypes \
	-Wmissing-declarations -Werror=undef -Wno-unused-function ${CMAKE_C_FLAGS}")

set(CMAKE_CXX_FLAGS "-std=gnu++17 -O3 -Werror -Wall -Wextra -Wuninitialized \
	-Wunused-variable -Wdouble-promotion \
	-Wmissing-declarations -Werror=undef -Wno-unused-function ${CMAKE_CXX_FLAGS}")

# Tell the linker not to worry about missing symbols in libraries (Linux only)
if(NOT APPLE)
	set(CMAKE_C_FLAGS   "-Wl,--unresolved-symbols=ignore-in-shared-libs ${CMAKE_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "-Wl,--unresolved-symbols=ignore-in-shared-libs ${CMAKE_CXX_FLAGS}")
endif()

# Platform-specific settings
if(PLATFORM STREQUAL QRB5165_2)
	message(STATUS "Building for platform QRB5165 2.X system images")
	add_definitions(-DPLATFORM_QRB5165_2)
elseif(PLATFORM STREQUAL QRB5165)
	message(STATUS "Building for platform QRB5165 1.X system images")
	add_definitions(-DPLATFORM_QRB5165)
endif()

# Library install directory
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^aarch64")
	set(LIB_INSTALL_DIR /usr/lib64)
else()
	set(LIB_INSTALL_DIR /usr/lib)
endif()

# Ignore some warnings when building 32-bit applications for VOXL 2
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
	string(REPLACE "-Wmissing-declarations" "" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
endif()

# Include subdirectories
add_subdirectory(src)