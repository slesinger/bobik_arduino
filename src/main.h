#ifndef main_h
#define main_h

#include <stdint.h>

const unsigned int MAIN_LOOP_FREQ_MS = 50; //[ms] = 1000[ms]/freq[Hz] target is 10ms
#define WAIT_GRANULARITY 1 //[ms]
uint8_t log_buf[64]; //uses only 4 bytes hower to avoid buffer overrun

#endif
