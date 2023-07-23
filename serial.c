/*
 * serial.c
 *
 * Created: 21/07/2023
 * Author : Richard Tomlinson G4TGJ
 */ 

#include <stdio.h>
#include "serial.h"
#include "config.h"

void serialInit( uint32_t baud)
{
    // Ignore the baud rate as we are using USB serial
    (void) baud;
    
    stdio_init_all();
}

void serialTransmit( uint8_t data )
{
    putchar_raw( data );
}

// Send a string ending with a NULL
void serialTXString( char *string )
{
    puts_raw( string );
}

uint8_t serialReceive( void )
{
    int c = getchar_timeout_us(1);

    // If there is no character then return NUL
    if( c == PICO_ERROR_TIMEOUT )
    {
        c = 0;
    }

    return c;
}
