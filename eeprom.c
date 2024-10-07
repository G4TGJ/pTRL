/*
 * eeprom.c
 *
 * Simulate an EEPROM using the last flash sector.
 * 
 * Flash is written in 256 byte pages but
 * erased in 4096 byte sectors.
 * * 
 * Generally we shouldn't call this code directly
 * but via the nvram code.
 *
 * Created: 15/07/2023
 * Author : Richard Tomlinson G4TGJ
 */ 
 
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "config.h"

// Disabled if we have an external I2C EEPROM
#ifndef EEPROM_I2C_ADDRESS

// The EEPROM size is configured in config.h
// Check it isn't too large. An address of 0xFF means the flash
// is empty so can only have up to 254 bytes.
#if EEPROM_SIZE > 254
    #error EEPROM_SIZE too large
#endif

// Cached copy of the EEPROM
// All reads are from here (once loaded from the flash)
static uint8_t eeprom[EEPROM_SIZE];

// Copy of the current flash page.
static uint8_t page[FLASH_PAGE_SIZE];

// There are 16 pages (4096/256)
#define NUM_PAGES (FLASH_SECTOR_SIZE/FLASH_PAGE_SIZE)

// The 4096 byte flash sector is at the very end of flash
#define EEPROM_FLASH_BASE_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

// We store EEPROM data in the flash as 2 byte structures containing
// an address and the value. We only write records if data changes.
// Most updates to the NVRAM are a single byte plus the checksum to make a
// total of 3 bytes. This will write 6 bytes to the flash. So our 4096 byte
// sector will need erasing approximately every 700 writes. It has an estimated
// life of 100000 erases so that's 70 million which compares favourably with an
// EEPROM which has a life of 1 million.
struct eepromEntry
{
    uint8_t address;
    uint8_t value;
};

// Pointer to the page buffer as a pointer to the entries
struct eepromEntry *pageEntries = (struct eepromEntry *) page;

// The current page number in the flash
static int pageNum;

// The next entry number in the page we will write to
static int entryNum;

// The number of entries in a page
#define NUM_ENTRIES (FLASH_PAGE_SIZE/sizeof(struct eepromEntry))

// Have we initialised the eeprom driver yet?
static bool bInit = false;

void eepromInit(void)
{
    int i;

    // We are going to read in the data from the flash so we need
    // to start with a blank EEPROM i.e. all 0xFF
    memset( eeprom, 0xFF, EEPROM_SIZE );

    // Read each page of the flash
    bool bFinished = false;
    for( pageNum = 0 ; pageNum < NUM_PAGES ; pageNum++ )
    {
        // Copy the current page into the page buffer
        memcpy( page, (uint8_t *) (XIP_BASE + EEPROM_FLASH_BASE_OFFSET + pageNum * FLASH_PAGE_SIZE), FLASH_PAGE_SIZE);

        // Read in each data entry in the page
        for( i = 0 ; i < NUM_ENTRIES ; i++)
        {
            // If the address read in is 0xFF then we have finished reading
            if( pageEntries[i].address == 0xFF )
            {
                // This is the next entry we will write to
                entryNum = i;
                bFinished = true;
                break;
            }

            // If the address is not too large then write the value into the eeprom
            else if( pageEntries[i].address < EEPROM_SIZE )
            {
                eeprom[pageEntries[i].address] = pageEntries[i].value;
            }
        }

        // Stop going through the pages once we have finished
        if( bFinished )
        {
            break;
        }
    }
    bInit = true;
}

uint8_t eepromRead(uint16_t uiAddress)
{
    // Need to initialise the driver the first time
    if( !bInit )
    {
        eepromInit();
    }

    // Read from our copy of the eeprom
    return eeprom[uiAddress];
}

// Program the page buffer into the correct page in flash
static void programFlash( int pagenum )
{
    uint32_t flags = save_and_disable_interrupts();
    flash_range_program(EEPROM_FLASH_BASE_OFFSET + (pageNum * FLASH_PAGE_SIZE), page, FLASH_PAGE_SIZE);
    restore_interrupts(flags);
}

// Erase the flash sector
static void eraseFlash( void )
{
    uint32_t flags = save_and_disable_interrupts();
    flash_range_erase(EEPROM_FLASH_BASE_OFFSET, FLASH_SECTOR_SIZE);
    restore_interrupts(flags);
}

// Store an eeprom entry in the flash
static void storeEntry(uint16_t entry, uint16_t uiAddress, uint8_t ucData)
{
    pageEntries[entry].address = uiAddress;
    pageEntries[entry].value = ucData;
}

void eepromWrite(uint16_t uiAddress, uint8_t ucData)
{
    // Only continue if the address is valid and the data has changed
    if( uiAddress < EEPROM_SIZE && eeprom[uiAddress] != ucData )
    {
        // Update our copy of the eeprom
        eeprom[uiAddress] = ucData;

        // Write the next entry into our page buffer
        storeEntry( entryNum, uiAddress, ucData );
        entryNum++;

        // Program the page into flash. This will actually only affect the two bytes we have
        // just written.
        programFlash( pageNum );

        // If we have finished the page then move onto the next one
        if( entryNum >= NUM_ENTRIES )
        {
            entryNum = 0;
            pageNum++;

            // Start with a blank page buffer
            memset( page, 0xFF, FLASH_PAGE_SIZE );
        }

        // If we got to the end of the flash sector then need to erase and start again
        if( pageNum >= NUM_PAGES )
        {
            pageNum = 0;
            entryNum = 0;

            // Erase the whole of the flash sector
            eraseFlash();

            // We now need to write the eeprom data back into our page buffer
            for( int i = 0 ; i < EEPROM_SIZE ; i++ )
            {
                // Blank bytes don't need to be written
                if( eeprom[i] != 0xFF )
                {
                    storeEntry( entryNum, i, eeprom[i] );
                    entryNum++;
                }
            }

            // Program this first page into the flash
            programFlash( 0 );
        }
    }
}
#endif
