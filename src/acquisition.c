#include <math.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include "acquisition.h"
#include "radio_interface.h"

#define RECORDING_BUFFER_SIZE     (size_t) 16777216
#define CALIBRATION_BUFFER_NUMBER (uint8_t) 20
#define CALIBRATION_FACTOR        (double) 4.f

typedef enum {
    ACQ_INIT = 0,
    ACQ_CALIBRATE,
    ACQ_LISTEN,
    ACQ_RECORD,
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
    atomic_bool acquisition_is_paused;

    // Public data
    uint8_t     recording_buf[RECORDING_BUFFER_SIZE];
    uint32_t    recording_buf_idx;
    sem_t       recording_sem;

} acquisition_data_t;

static acquisition_data_t acq_data;

/***********************************************************************************************************************
 * Private functions declaration
 **********************************************************************************************************************/
static void threshold_calibration(const uint8_t *iq, size_t len);
static double median(double *values, size_t n);
static int compare(const void *a, const void *b);
static void listen(const uint8_t *iq, size_t len);
static void record(const uint8_t *iq, size_t len);
static void handle_pause(const uint8_t *iq, size_t len);
static void stop_recording(void);

/***********************************************************************************************************************
 * Public functions implementation
 **********************************************************************************************************************/

/**
 * @brief Initialize the parameters and attach callback function to radio_interface.
 * @return 0 if we went through the function
 */
int acquisition_init(void)
{
    acq_data.state = ACQ_INIT;
    acq_data.calib_buf_idx = 0;
    acq_data.calib_factor = CALIBRATION_FACTOR;
    radio_set_callback(on_iq_samples, &acq_data);
    sem_init(&acq_data.recording_sem, 0, 0);
    return 0;
}

/**
 * @brief Callback function called by the radio_interface, who provides the IQ samples.
 * @param iq  (in) the buffer of iq samples
 * @param len (in) the length of the iq buffer
 * @param user_ctx (in/out) context, unused yet
 */
static void on_iq_samples(const uint8_t *iq, size_t len, void *user_ctx)
{
    // acquisition_ctx_t *acquisition_ctx = (acquisition_data_t *) user_ctx;

    switch (acq_data.state)
    {
        case ACQ_INIT:
            acq_data.state = ACQ_CALIBRATE;
            printf("Starting calibration...\n");
        case ACQ_CALIBRATE:
            threshold_calibration(iq, len);
            break;
        case ACQ_LISTEN:
            listen(iq, len);
            break;
        case ACQ_RECORD:
            record(iq, len);
            break;
        case ACQ_PAUSE:
            handle_pause(iq, len);
            break;
        default:
            break;
    }
}

/**
 * @brief This function is meant to be used exclusively by the processing module.
 * It informs the acquisition module that the processing is over, and that he can start listening the signal
 * again.
 */
void unpause_acquisition(void)
{
    acq_data.acquisition_is_paused = false;
}

/**
 * @brief Used by the processing module to get the address of the recording buffer.
 * This allows the processing module to read directly the recording buffer, and avoid to waste time copying data.
 * We prevent race condition with a semaphore.
 * @return the address of the reocrding buffer
 */
uint8_t* get_acquisition_buffer(void)
{
    return acq_data.recording_buf;
}

/**
 * @brief Used by the processing module to get the address of the semaphore.
 * The semaphore is used to announce to the processing module that the recording buffer is ready to be read.
 * @return the address
 */
sem_t* get_acquisition_semaphore(void)
{
    return &acq_data.recording_sem;
}

/**
 * @brief This functions is used by the processing module to know how many iq samples he must read in the recording
 * buffer
 * @param length (in/out) is set to the value of acq_data.recording_buf_idx
 */
void get_buffer_length(uint32_t *length)
{
    *length = acq_data.recording_buf_idx;
}

/**
 * @brief compute the mean power of the signal. This signal is expected to be an buffer of IQ samples (complex numbers)
 * @param iq  (in) the buffer of iq samples
 * @param len (in) the length of the iq buffer
 * @return void
 */
double mean_power(const uint8_t *iq, const size_t len)
{
    if (iq == NULL || len < 2)
    {
        return 0.0;
    }

    double sum = 0;
    for (size_t i=0; i+1 < len; i+=2)
    {
        double I = iq[i]   - 127.5;
        double Q = iq[i+1] - 127.5;
        sum += I*I + Q*Q;
    }

    return sum / (double) len /2;;
}

/***********************************************************************************************************************
 * Private functions implementations
 **********************************************************************************************************************/

/**
 * @brief compute the mean power of N buffers. Then set the threshold to the median value of these mean powers
 * @param iq  (in) the buffer of iq samples
 * @param len (in) the length of the iq buffer
 * @return void
 */
static void threshold_calibration(const uint8_t *iq, size_t len)
{
    acq_data.calib_buf[acq_data.calib_buf_idx] = mean_power(iq, len);
    acq_data.calib_buf_idx++;

    if (acq_data.calib_buf_idx >= CALIBRATION_BUFFER_NUMBER - 1)
    {
        double median_value = median(acq_data.calib_buf, CALIBRATION_BUFFER_NUMBER);
        acq_data.threshold = acq_data.calib_factor * median_value;
        acq_data.state = ACQ_LISTEN;
        printf("Calibration done.\n");
        // printf("Noise level estimated to %lf. Threshold set to %lf\n", median_value, acq_data.threshold);
        printf("Now listening.\n");
    }
}

/**
 * @brief return the median value of a serie, using a quick sort algorithm from stdlib
 * @param values (in) the serie of points
 * @param n      (in) the size of the serie
 * @return the median value
 */
static double median(double *values, size_t n)
{
    qsort(values, n, sizeof(double), compare);

    if (n % 2 != 0)
        return values[n / 2];

    return (values[n / 2 - 1] + values[n / 2]) / 2.0;
}

/**
 * @brief compare two double values
 * @param a (in) the first value
 * @param b (in) the second value
 * @return 1 if a>b, -1 if a<b, 0 if a==b
 */
static int compare(const void *a, const void *b)
{
    double da = *(const double *)a;
    double db = *(const double *)b;
    if (da < db) return -1;
    if (da > db) return 1;
    return 0;
}

/**
 * @brief Compares the received mean power of the signal and the detection threshold.
 * If the value is below, the buffer is simply discarded.
 * If the value is above, it triggers the recording of the signal.
 * @param iq  (in) the buffer of iq samples
 * @param len (in) the length of the iq buffer
 * @return void
 */
static void listen(const uint8_t *iq, size_t len)
{
    if (mean_power(iq, len) < acq_data.threshold)
    {
        return;
    }
    printf("New signal detected. Starting recording.\n");
    if (len <= RECORDING_BUFFER_SIZE)
    {
        acq_data.state = ACQ_RECORD;
        memcpy(acq_data.recording_buf, iq, len);
        acq_data.recording_buf_idx = len;
    }
}

/**
 * @brief Record the signal until its mean power goes below the threshold again, or until the recording buffer is full.
 * @param iq  (in) the buffer of iq samples
 * @param len (in) the length of the iq buffer
 * @return void
 */
static void record(const uint8_t *iq, size_t len)
{
    size_t remaining = RECORDING_BUFFER_SIZE - acq_data.recording_buf_idx;

    if (mean_power(iq, len) < acq_data.threshold || len > remaining)
    {
        stop_recording();
        return;
    }

    memcpy(acq_data.recording_buf + acq_data.recording_buf_idx, iq, len);
    acq_data.recording_buf_idx += len;

    if (acq_data.recording_buf_idx >= RECORDING_BUFFER_SIZE)
    {
        stop_recording();
    }
}

/**
 * @brief Set the acquisition in a pause state: all incoming buffer will then be discarded untill the processing module
 * set acq_data.recording_is_paused back to false.
 */
static void stop_recording(void)
{
    acq_data.state = ACQ_PAUSE;
    acq_data.acquisition_is_paused = true;
    sem_post(&acq_data.recording_sem);
    printf("Recording done.\n");
}

/**
 * @brief This functions handles the incoming buffer while the acquisition is paused (which happens after a recording
 * was done and until the signal processing of the said recording is over).
 * If the acquisition is paused the buffer is discarded.
 * Else it compares the mean power of the received signal with the detection threshold.
 * If the value is below the threshold, it simply switches to the listening state.
 * If the value is above the threshold, it starts a new recording.
 * @param iq  (in) the buffer of iq samples
 * @param len (in) the length of the iq buffer
 * @return void
 */
static void handle_pause(const uint8_t *iq, size_t len)
{
    if (acq_data.acquisition_is_paused)
    {
        return;
    }

    printf("Acquisition unpaused.\n");
    acq_data.recording_buf_idx = 0;

    if (mean_power(iq, len) < acq_data.threshold)
    {
        printf("Now listening.\n");
        acq_data.state = ACQ_LISTEN;
        return;
    }

    printf("New signal detected. Starting recording.\n");
    acq_data.state = ACQ_RECORD;
    memcpy(acq_data.recording_buf, iq, len);
    acq_data.recording_buf_idx += len;
}