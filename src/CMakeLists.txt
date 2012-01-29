set(FIRMWARE_NAME loguino)

set(${FIRMWARE_NAME}_BOARD mega2560)
set(${FIRMWARE_NAME}_SRCS loguino.cpp)

generate_arduino_firmware(loguino)
