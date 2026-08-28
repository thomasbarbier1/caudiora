#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include "rtl-sdr.h"
#include "rtlsdr_interface.h"

#define CENTER_FREQUENCY     (uint32_t)868300000
#define SAMPLE_RATE          (uint32_t)2400000
#define BANDWIDTH            (uint32_t)125000

rtlsdr_context_t radio_context = {
    .lock = PTHREAD_MUTEX_INITIALIZER,
    .cv   = PTHREAD_COND_INITIALIZER,
    .new_data_is_available = false,
    .running = true,
    .data = 0,
    .data_len = 0
};

rtlsdr_dev_t *radio_device;

int main(void)
{
    const rtlsdr_config_t config =
    {
        .central_frequency = CENTER_FREQUENCY,
        .bandwidth         = BANDWIDTH,
        .sample_rate       = SAMPLE_RATE,
    };

    if (rtlsdr_init(&config) != 0)
    {
        printf("Error: unable to configure radio device\n");
    }


    rtlsdr_stream_start();
    rtlsdr_stream_stop();

    printf("End of program.\n");
    return 0;
}



