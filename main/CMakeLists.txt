set(srcs "main.c")

set(requires "bluepad32")

# See: https://gitlab.com/ricardoquesada/bluepad32/-/issues/9
if(DEFINED ENV{BLUEPAD32_ARDUINO})
    list(APPEND srcs "arduino_main.cpp")
    list(APPEND requires "bluepad32_arduino" "arduino" "espsoftwareserial" "LiquidCrystal_I2C" "Adafruit_SSD1306" "Adafruit-GFX-Library" "crc32" "Arduino-PID-Library" "Arduino-PID-AutoTune-Library" "memory_system_data" "CRSF" "simulator")
endif()

idf_component_register(SRCS "lookup_table_types.c" "lookup-tables_30_10_60_100.c" "syscommands.cpp" "bobbycar.cpp" "pushed_follower.cpp" "rc_manager.cpp" "logging.cpp" "display_oled.cpp" "display.cpp" "display_2004.cpp" "display_none.cpp" "pid_contols.cpp" "string_tools.c" "command_interpreter.c" "console_task.c" "inputreader.c" "gamepad_manager.cpp" "adc1_read.c" "math_functions.c" "bobbycar.hpp" "${srcs}"
                    INCLUDE_DIRS "."
                    REQUIRES "${requires}")
