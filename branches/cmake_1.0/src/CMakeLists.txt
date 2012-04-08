set(FIRMWARE_NAME loguino)
set(loguino_RECURSE True)
set(SD_RECURSE True)

set(${FIRMWARE_NAME}_BOARD mega2560)
set(${FIRMWARE_NAME}_SRCS loguino.cpp)
set(${FIRMWARE_NAME}_PORT /dev/tty.usbmodemfd121)

generate_arduino_firmware(loguino)
