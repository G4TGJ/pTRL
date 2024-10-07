/*
 * eeprom_i2c.c
 *
 * Driver for external EEPROM device over I2C
 * 
 * Implements wear levelling to give very long
 * life from device. Presents a small EEPROM to the user
 * ~256 bytes but spreads it over the whole 64K device.
 * Also allows for running I2C at only 100kHz.
 * Although the device can probably run at 1MHz other
 * devices on the bus may not work at this speed so
 * it minimises how much needs to be read.
 * 
 * Divides the device into pages. The final page is an
 * index page. It points to the current page. This page
 * contains a copy of the EEPROM at the start. When there
 * is a write it stores the address and data byte successively
 * in the page. When it gets to the end of the page it moves to
 * the next page and writes a new index.
 * 
 * Generally we shouldn't call this code directly
 * but via the nvram code.
 *
 * Created: 11/02/2024
 * Author : Richard Tomlinson G4TGJ
 */ 
 
#include <inttypes.h>
#include <string.h>

#include "config.h"
#include "eeprom.h"
#include "i2c.h"
#include "io.h"

// Only enabled if I2C address set
#ifdef EEPROM_I2C_ADDRESS

// There are 64 pages (65536/1024)
#define NUM_PAGES (EEPROM_DEVICE_SIZE/EEPROM_PAGE_SIZE)

// The last page is number 62 as number 63 is the index page
#define LAST_PAGE (NUM_PAGES - 2)

// The index page is the last page
#define INDEX_PAGE_ADDRESS (EEPROM_DEVICE_SIZE - EEPROM_PAGE_SIZE)

// The EEPROM size is configured in config.h
// Check it isn't too large. An address of 0xFF means the flash
// is empty so can only have up to 255 bytes.
#if EEPROM_SIZE > 255
    #error EEPROM_SIZE too large
#endif

// Cached copy of the EEPROM
// All reads are from here (once loaded from the flash)
static uint8_t eeprom[EEPROM_SIZE];

// Copy of the current page
static uint8_t page[EEPROM_PAGE_SIZE];

// The index page
static uint8_t indexPage[EEPROM_PAGE_SIZE];

// We store EEPROM data in the device as 2 byte structures containing
// an address and the value. We only write records if data changes.
// Most updates to the NVRAM are a single byte plus the checksum to make a
// total of 3 bytes. This will write 6 bytes to the flash. 
struct eepromEntry
{
    uint8_t address;
    uint8_t value;
};

// Pointer to the entries in the page buffer
// They are after the initial copy of the EEPROM
static struct eepromEntry *pageEntries = (struct eepromEntry *) (page + EEPROM_SIZE);

// The current page number
static int pageNum;

// The next entry number in the page we will write to
static int entryNum;

// The current position in the index page
static int indexPos;

// The number of entries in a page
// The page starts with an initial copy of the EEPROM so allow for that
#define NUM_ENTRIES ((EEPROM_PAGE_SIZE - EEPROM_SIZE)/sizeof(struct eepromEntry))

// Functions to read and write from the device itself
static uint8_t deviceRead(uint16_t address)
{
    uint8_t data;
    i2cReadRegisterA16(EEPROM_I2C_ADDRESS, address, &data);
    return data;
}

static void deviceReadNum(uint16_t address, uint8_t *buffer, uint16_t num)
{
    i2cReadDataA16( EEPROM_I2C_ADDRESS, address, buffer, num);
}

static void deviceWrite(uint16_t address, uint8_t data)
{
    i2cWriteRegisterA16(EEPROM_I2C_ADDRESS, address, data);

    // Wait for the write to complete
    i2cWait( EEPROM_I2C_ADDRESS );
}

static void deviceWriteNum(uint16_t address, uint8_t *buffer, uint16_t num)
{
    // Each write to the EEPROM has to be within a single page
    // Keep track of how many bytes are left to write
    int bytesLeft = num;

    // The position within the buffer
    int pos = 0;

    // The current address to write to in the EEPROM
    uint16_t currentAddress = address;

    while( bytesLeft > 0 )
    {
        // Get the length to write to the end of the page
        int len = EEPROM_DEVICE_PAGE_SIZE - (currentAddress & EEPROM_DEVICE_PAGE_MASK);

        // Obviously don't write any more bytes than there is data
        if( len > bytesLeft )
        {
            len = bytesLeft;
        }
        i2cWriteDataA16( EEPROM_I2C_ADDRESS, currentAddress, &buffer[pos], len);

        // Wait for the write to complete
        i2cWait( EEPROM_I2C_ADDRESS );

        bytesLeft -= len;
        pos += len;
        currentAddress += len;
    }
}

void eepromInit(void)
{
    int i;

    i2cInit();

    // We are going to read in the data from the flash so we need
    // to start with a blank EEPROM i.e. all 0xFF
    memset( eeprom, 0xFF, EEPROM_SIZE );

    // Read the index page
    deviceReadNum( INDEX_PAGE_ADDRESS, indexPage, EEPROM_PAGE_SIZE );

    // Find the current index - we search from the start of the page
    // As soon as we find one that's invalid we stop.
    for( i = 0 ; i < EEPROM_PAGE_SIZE ; i++ )
    {
        if( indexPage[i] > LAST_PAGE )
        {
            break;
        }
    }

    // If the first entry was invalid then we treat the device as blank
    if( i == 0 )
    {
        indexPos = 0;

        // Start at the first page
        pageNum = 0;

        // Write the first page num into the index
        deviceWrite(INDEX_PAGE_ADDRESS, 0);

        // The second byte is blank as no more records
        deviceWrite(INDEX_PAGE_ADDRESS + 1, 0xFF);

        // Ensure the first page is sufficiently blank - that's the EEPROM plus
        // the first update address
        deviceWriteNum(0, eeprom, EEPROM_SIZE);
        deviceWrite(EEPROM_SIZE, 0xFF);
    }
    else
    {
        // The page in use is the one stored before the invalid index
        indexPos = i - 1;
        pageNum = indexPage[indexPos];

        // Read in the current page
        deviceReadNum(pageNum * EEPROM_PAGE_SIZE, page, EEPROM_PAGE_SIZE);

        // Copy the EEPROM from the start of this page
        memcpy(eeprom, page, EEPROM_SIZE);

        // Read in each data entry in the page
        for( i = 0 ; i < NUM_ENTRIES ; i++ )
        {
            // If the address read in is 0xFF then we have finished reading
            if( pageEntries[i].address == 0xFF )
            {
                // This is the next entry we will write to
                entryNum = i;
                break;
            }

            // If the address is not too large then write the value into the eeprom
            else if( pageEntries[i].address < EEPROM_SIZE )
            {
                eeprom[pageEntries[i].address] = pageEntries[i].value;
            }
        }
    }
}

uint8_t eepromRead(uint16_t address)
{
    // Read from our copy of the eeprom
    return eeprom[address];
}

// Store an eeprom entry into the EEPROM
static void storeEntry(uint16_t entry, uint8_t address, uint8_t data)
{
    // Write the address and data followed by a blank byte
    uint8_t buf[3] = { address, data, 0xFF };

    // The position in the EEPROM is the relevant entry on the current page starting
    // after the initial copy of the EEPROM.
    uint16_t pos = pageNum*EEPROM_PAGE_SIZE + entry*sizeof(pageEntries[0]) + EEPROM_SIZE;
    int len;

    // Need to be sure that the final blank byte doesn't overwrite the index page
    if( (pos + 2) < INDEX_PAGE_ADDRESS )
    {
        len = 3;
    }
    else
    {
        len = 2;
    }
    deviceWriteNum( pos, buf, len );
}

void eepromWrite(uint16_t address, uint8_t data)
{
    // Only continue if the address is valid and the data has changed
    if( address < EEPROM_SIZE && eeprom[address] != data )
    {
        // Update our copy of the eeprom
        eeprom[address] = data;

        // Write the next entry into our page buffer
        storeEntry( entryNum, address, data );
        entryNum++;

        if( entryNum >= NUM_ENTRIES )
        {
            // If we have finished the page then move onto the next one
            entryNum = 0;
            pageNum++;
            if( pageNum > LAST_PAGE )
            {
                pageNum = 0;
            }

            // Write the cached eeprom to the start of the page
            deviceWriteNum( pageNum * EEPROM_PAGE_SIZE, eeprom, EEPROM_SIZE );

            // Write the new page number into the next index position
            indexPos++;
            if( indexPos >= EEPROM_PAGE_SIZE )
            {
                indexPos = 0;
            }
            deviceWrite( INDEX_PAGE_ADDRESS + indexPos, pageNum );

            // Also need to ensure the next entry is blank
            // If it's the last entry there's no need to erase the next byte
            if( indexPos < EEPROM_PAGE_SIZE - 1 )
            {
                deviceWrite( INDEX_PAGE_ADDRESS + indexPos + 1, 0xFF );
            }
        }
    }
}
#endif
