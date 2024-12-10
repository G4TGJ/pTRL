#include <stdio.h>

#include "pico/stdlib.h"
#include "config.h"

void millisInit()
{
}

// Return the current millisecond tick count
uint32_t millis ()
{
    return time_us_64() / 1000;
}