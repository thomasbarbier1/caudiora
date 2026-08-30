#include <math.h>
#include <stdio.h>
#include "acquisition.h"
#include "radio_interface.h"

typedef enum {
    ACQ_INIT = 0,
    ACQ_CALIBRATING,
    ACQ_LISTENING,
    ACQ_RECORDING,
    ACQ_PAUSE
} acq_state_e;

typedef struct
{
    acq_state_e state;
    uint8_t detection_threshold;
    uint8_t recording_buffer[RECORDING_BUFFER_SIZE];
} acquisition_data_t;

static acquisition_data_t acquisition_data;

int acquisition_init(void)
{
    acquisition_data.state = ACQ_INIT;
    radio_set_callback(on_iq_samples, &acquisition_data);
    return 0;
}

static void on_iq_samples(const uint8_t *iq, size_t len, void *user_ctx)
{
    acquisition_data_t *acquisition_data = (acquisition_data_t *) user_ctx;

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


