# Skip pkg-config for cross-compilation, manually set paths
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)
include_directories(/usr/include)

# main executable
add_executable(voxl-mavlink-mqtt-client
	main.cpp
	mqtt_client.cpp
	config_file.cpp
	mavlink_json.cpp
	publish_timer.cpp
)

# link libraries
target_link_libraries(voxl-mavlink-mqtt-client
	pthread
)

# Handle mosquitto linking based on build type
if(CMAKE_CROSSCOMPILING OR DEFINED CMAKE_TOOLCHAIN_FILE)
    # Cross-compilation: use ARM64 mosquitto and ensure 64-bit library paths
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L/usr/lib/aarch64-linux-gnu -L/usr/lib64")
    target_link_libraries(voxl-mavlink-mqtt-client mosquitto)

    # Link VOXL libraries for cross-compilation
    target_link_libraries(voxl-mavlink-mqtt-client
        modal_pipe
        modal_json
        voxl_cutils
        mavlink-to-json
    )
else()
    # Native build: use x86_64 mosquitto and skip VOXL libraries
    target_link_libraries(voxl-mavlink-mqtt-client mosquitto)
    message(STATUS "Native build: Skipping VOXL libraries")
endif()

# install executable
install(TARGETS voxl-mavlink-mqtt-client DESTINATION /usr/bin)