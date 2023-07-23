#define delayMicroseconds sleep_us
#define delay sleep_ms

// Initialise the timer
void millisInit();

// Return the current time in milliseconds
uint32_t millis(void);
