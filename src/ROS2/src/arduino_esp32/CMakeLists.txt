cmake_minimum_required(VERSION 3.16)
project(arduino_esp32 NONE)

find_package(ament_cmake REQUIRED)

# PlatformIO is provided by the Nix dev shell (see flake.nix).
find_program(PIO_EXECUTABLE pio
  DOC "PlatformIO core CLI (provided by nix develop)")
if(NOT PIO_EXECUTABLE)
  message(FATAL_ERROR
    "PlatformIO 'pio' not found in PATH. Enter the dev shell first: nix develop")
endif()

set(PIO_PROJECT_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(PIO_ENV "featheresp32-v2")
set(PIO_BUILD_DIR "${PIO_PROJECT_DIR}/.pio/build/${PIO_ENV}")
set(FIRMWARE_BIN "${PIO_BUILD_DIR}/firmware.bin")

# All `pio` invocations must go through pio_clean.sh, which strips Nix/ROS
# environment leaks that would otherwise corrupt micro_ros_platformio's
# nested host-side build. See that script's header comment for details.
set(PIO_RUN ${PIO_PROJECT_DIR}/scripts/pio_clean.sh)

# Build firmware as part of `colcon build` (added to ALL).
add_custom_target(firmware ALL
  COMMAND ${PIO_RUN} run -d ${PIO_PROJECT_DIR} -e ${PIO_ENV}
  WORKING_DIRECTORY ${PIO_PROJECT_DIR}
  COMMENT "Building micro-ROS firmware for ${PIO_ENV} via PlatformIO"
  USES_TERMINAL
  VERBATIM)

# Refresh compile_commands.json on every build so clangd (and other LSPs)
# can find Arduino/micro-ROS/Wire headers and the Xtensa flags PlatformIO
# emits. PlatformIO writes the file directly into ${PIO_PROJECT_DIR},
# which is where clangd's default upward search picks it up.
add_custom_target(compiledb ALL
  COMMAND ${PIO_RUN} run -d ${PIO_PROJECT_DIR} -e ${PIO_ENV} -t compiledb
  BYPRODUCTS ${PIO_PROJECT_DIR}/compile_commands.json
  WORKING_DIRECTORY ${PIO_PROJECT_DIR}
  COMMENT "Refreshing ${PIO_PROJECT_DIR}/compile_commands.json (for clangd)"
  USES_TERMINAL
  VERBATIM)
add_dependencies(compiledb firmware)

# Flash target — explicitly NOT in ALL. Invoke manually:
#   cmake --build build/arduino_esp32 --target flash
# Honors ESP32_SERIAL_PORT (default /dev/ttyUSB0).
add_custom_target(flash
  COMMAND ${CMAKE_COMMAND} -E env
          "ESP32_SERIAL_PORT=$ENV{ESP32_SERIAL_PORT}"
          ${PIO_PROJECT_DIR}/scripts/flash.sh
  WORKING_DIRECTORY ${PIO_PROJECT_DIR}
  COMMENT "Flashing firmware to ESP32 (port: $ENV{ESP32_SERIAL_PORT}, default /dev/ttyUSB0)"
  USES_TERMINAL
  VERBATIM)
add_dependencies(flash firmware)

# Stage the built firmware into the colcon install tree so other tooling
# (and the user) can find a stable path to the artifact.
install(FILES ${FIRMWARE_BIN}
  DESTINATION share/${PROJECT_NAME}/firmware
  OPTIONAL)

install(PROGRAMS scripts/flash.sh
  DESTINATION lib/${PROJECT_NAME})

ament_package()
