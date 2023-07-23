add_library(pTRL INTERFACE)

target_sources(pTRL INTERFACE
    eeprom.c
    i2c.c
    millis.c
    millis.h
    serial.c
)

target_include_directories(pTRL INTERFACE . ${CMAKE_SOURCE_DIR})
target_link_libraries(pTRL INTERFACE
    hardware_adc
    hardware_i2c
    hardware_pwm
)

add_library(TRL_lcd_port INTERFACE)
target_sources(TRL_lcd_port INTERFACE lcd_port.c)
target_include_directories(TRL_lcd_port INTERFACE . ${CMAKE_SOURCE_DIR})
