#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include "acquisition.h"
#include "radio_interface.h"

#define CALIBRATION_BUFFER_NUMBER (uint8_t) 100
#define CALIBRATION_FACTOR        (double) 4.f

typedef enum {
    ACQ_INIT = 0,
    ACQ_CALIBRATING,
    ACQ_LISTENING,
    ACQ_RECORDING,
    ACQ_PAUSE
} acq_state_e;

typedef struct
{
    // Private data
    acq_state_e state;
    double      threshold;
    double      calib_buf[CALIBRATION_BUFFER_NUMBER];
    uint8_t     calib_buf_idx;
    double      calib_factor;

    // Public data
    uint8_t recording_buffer[RECORDING_BUFFER_SIZE];

} acquisition_data_t;

static acquisition_data_t acq_data;

/***********************************************************************************************************************
 * Private functions declaration
 **********************************************************************************************************************/
static void threshold_calibration(const uint8_t *iq, const size_t len);
static double mean_power(const uint8_t *iq, const size_t len);
static double median(double *values, size_t n);
static int compare(const void *a, const void *b);

/***********************************************************************************************************************
 * Public functions implementation
 **********************************************************************************************************************/
int acquisition_init(void)
{
    acq_data.state = ACQ_INIT;
    acq_data.calib_buf_idx = 0;
    acq_data.calib_factor = CALIBRATION_FACTOR;
    radio_set_callback(on_iq_samples, &acq_data);
    return 0;
}

static void on_iq_samples(const uint8_t *iq, const size_t len, void *user_ctx)
{
    // acquisition_ctx_t *acquisition_ctx = (acquisition_data_t *) user_ctx;

    switch (acq_data.state)
    {
        case ACQ_INIT:
            acq_data.state = ACQ_CALIBRATING;
            printf("Starting calibration...\n");
        case ACQ_CALIBRATING:
            threshold_calibration(iq, len);
            break;
        case ACQ_LISTENING:
            // printf("mean power: %lf\n", mean_power(iq, len));
            break;
        case ACQ_RECORDING:
            break;
        case ACQ_PAUSE:
            break;

    }
}

/***********************************************************************************************************************
 * Private functions implementations
 **********************************************************************************************************************/

static void threshold_calibration(const uint8_t *iq, const size_t len)
{
    acq_data.calib_buf[acq_data.calib_buf_idx] = mean_power(iq, len);
    acq_data.calib_buf_idx++;

    if (acq_data.calib_buf_idx >= CALIBRATION_BUFFER_NUMBER - 1)
    {
        double median_value = median(acq_data.calib_buf, CALIBRATION_BUFFER_NUMBER);
        acq_data.threshold = acq_data.calib_factor * median_value;
        acq_data.state = ACQ_LISTENING;
        printf("Calibration done.\n");
        printf("Noise level estimated to %lf. Threshold set to %lf\n", median_value, acq_data.threshold);
        printf("Now listening.\n");
    }
}

static double mean_power(const uint8_t *iq, const size_t len)
{
    double sum = 0;
    for (uint32_t i=0; i<len-1; i+=2)
    {
        double I = iq[i]   - 127.5;
        double Q = iq[i+1] - 127.5;
        sum += I*I + Q*Q;
    }
    return sum / len /2;
}

static double median(double *values, size_t n)
{
    qsort(values, n, sizeof(double), compare);

    if (n % 2 != 0)
        return values[n / 2];

    return (values[n / 2 - 1] + values[n / 2]) / 2.0;
}

static int compare(const void *a, const void *b)
{
    double da = *(const double *)a;
    double db = *(const double *)b;
    if (da < db) return -1;
    if (da > db) return 1;
    return 0;
}



