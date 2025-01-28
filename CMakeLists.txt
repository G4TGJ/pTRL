add_library(pTRL INTERFACE)

pico_generate_pio_header(pTRL ${CMAKE_CURRENT_LIST_DIR}/i2s.pio)

target_sources(pTRL INTERFACE
    eeprom.c
    eeprom_i2c.c
    i2c.c
    millis.c
    millis.h
    serial.c
    ssd1306_i2c.c
    i2s.c
    WM8960.c)

target_include_directories(pTRL INTERFACE . ${CMAKE_SOURCE_DIR})
target_link_libraries(pTRL INTERFACE
    hardware_adc
    hardware_i2c
    hardware_pwm
    hardware_pio
    )

add_library(TRL_lcd_port INTERFACE)
target_sources(TRL_lcd_port INTERFACE lcd_port.c)
target_include_directories(TRL_lcd_port INTERFACE . ${CMAKE_SOURCE_DIR})
