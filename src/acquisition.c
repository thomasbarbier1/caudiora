#include "acquisition.h"

#include <math.h>
#include <stdio.h>

enum acquisition_state
{
    INIT = 0,
    CALIBRATING,
    LISTENING,
    RECORDING,
    PAUSE
};

typedef struct acquisition_data
{
    enum acquisition_state state;
    uint8_t detection_threshold;
    uint8_t recording_buffer[RECORDING_BUFFER_SIZE];
} acquisition_data_t;

static acquisition_data_t acquisition_data;

void acquisition_init(void)
{
    acquisition_data.state = INIT;
}

void acquisition_cb(const uint8_t *iq, size_t len, void *user_ctx)
{
    double sum = 0;
    for (int i=0; i<len-1; i++)
    {
        double I = (double) iq[i] - 127.5;
        double Q = (double) iq[i+1] - 127.5;
        sum += sqrt(I*I + Q*Q);
    }
    double mean = (double) sum / len / 2;
    printf("mean: %lf\n", mean);
}


