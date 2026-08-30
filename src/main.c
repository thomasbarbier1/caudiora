#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include "radio_interface.h"
#include "acquisition.h"

#define CENTER_FREQUENCY (uint32_t)868300000
#define SAMPLE_RATE      (uint32_t)2400000
#define BANDWIDTH        (uint32_t)125000

int main(void)
{
    if (radio_config(CENTER_FREQUENCY, BANDWIDTH, SAMPLE_RATE) != 0)
    {
        printf("Failed to configure the radio. End of program.\n");
        return 1;
    }

    if (radio_init() != 0)
    {
        printf("Failed to initialize radio parameters. End of program.\n");
        return 1;
    }

    if (acquisition_init() != 0)
    {
        printf("Failed to initialize acquisition. End of program.\n");
        return 1;
    }

    pthread_t radio_thread;
    pthread_create(&radio_thread, NULL, radio_stream_start, NULL);

    sleep(5);

    uint32_t overflow_nb = radio_stream_stop();
    pthread_join(radio_thread, NULL);
    printf("Number of buffer overflow: %u\n", overflow_nb);
    printf("End of program.\n");

    return 0;
}



