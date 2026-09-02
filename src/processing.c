#include <stdio.h>
#include <semaphore.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "processing.h"
#include "acquisition.h"
#include "radio_interface.h"

#define AUDIO_RECORDING_SIZE (size_t) 16777216
#define AMPLITUDE_FLOOR      (double) 1e-8

typedef struct
{
    // private
    atomic_bool running;
    double      amplitude_floor;
    // public
    uint8_t  *iq;
    uint32_t len;
    sem_t*   processing_sem;
    double   audio_buffer[AUDIO_RECORDING_SIZE];

} processing_t;

static processing_t processing_data;

/***********************************************************************************************************************
 * Private functions declaration
 **********************************************************************************************************************/

static int processing_pipeline(const uint8_t *iq, size_t len);
static double *zero_centering_and_double_conversion(const uint8_t *iq, size_t len);
static void normalize_complex_amplitude(double *iq, size_t len);
static double *compute_instantaneous_frequency(const double *iq, size_t len);

/***********************************************************************************************************************
 * Public functions implementation
 **********************************************************************************************************************/

int processing_init(void)
{
    processing_data.iq = get_acquisition_buffer();
    processing_data.processing_sem = get_acquisition_semaphore();
    processing_data.amplitude_floor = AMPLITUDE_FLOOR;

    if (processing_data.iq == NULL)
    {
        return 1;
    }

    processing_data.running = true;
    return 0;
}

int processing_stop(void)
{
    processing_data.running = false;
    sem_post(processing_data.processing_sem); // in case the semaphore is waiting, we unlock it first
    sem_destroy(processing_data.processing_sem);

    return 0;
}

void* processing_start(void *ctx)
{
    while (processing_data.running)
    {
        sem_wait(processing_data.processing_sem);
        get_buffer_length(&processing_data.len);
        double v = mean_power(processing_data.iq, processing_data.len);
        printf("v: %lf \t len: %u\n", v, processing_data.len);
        unpause_acquisition();
    }

    return NULL;
}

/***********************************************************************************************************************
 * Private functions implementations
 **********************************************************************************************************************/

static int processing_pipeline(const uint8_t *iq, size_t len)
{
    if (iq == NULL || len < 2 || len % 2 != 0 || len > AUDIO_RECORDING_SIZE)
    {
        return 1;
    }

    double* iq_as_double = zero_centering_and_double_conversion(iq, len);
    normalize_complex_amplitude(iq_as_double, len);
    double* freq = compute_instantaneous_frequency(iq_as_double, len);
    
    free(iq_as_double);
    free(freq);
}

static double *zero_centering_and_double_conversion(const uint8_t *iq, size_t len)
{
    double *iq_as_double = malloc(len * sizeof(double));
    for (size_t i = 0; i + 1 < len; i+=2)
    {
        iq_as_double[i]     = (double) iq[i]     - 127.5;
        iq_as_double[i + 1] = (double) iq[i + 1] - 127.5;
    }
    return iq_as_double;
}

static void normalize_complex_amplitude(double *iq, size_t len)
{
    /* First, finding the maximum magnitude */
    double I = iq[0];
    double Q = iq[1];
    double max_magnitude = sqrt(I*I + Q*Q);
    for (size_t i=2; i+1 < len; i+=2)
    {
        I = iq[i];
        Q = iq[i+1];
        double mag = sqrt(I*I + Q*Q);
        if (mag > max_magnitude)
        {
            max_magnitude = mag;
        }
    }

    /* Then we compare this max magnitude with an arbitrary amplitude floor
     * If above, we normalize. If below, we keep the magnitude as is. */
    if (max_magnitude < processing_data.amplitude_floor)
    {
        return;
    }

    for (size_t i = 0; i < len; i++)
    {
        iq[i] /= max_magnitude;
    }
}

static double *compute_instantaneous_frequency(const double *iq, size_t len)
{
    /* Estimate instantaneous frequency from the phase increment. */

    /*
     * Dans cette fonction on cherche à obtenir f[n], la frequence instannee
     *
     * Or f(t) = 1/(2*pi) * dp(t)/dt           avec p(t) la phase instantanée
     *
     * On peut approximer ça en discret par:
     *
     *     f[n] =   1/(2*pi) * (p[n]-p[n-1])/Te
     *          =  fe/(2*pi) * (p[n]-p[n-1])
     *
     * Il nous faut donc récupérer p[n]-p[n-1]. Au début j'ai essayé d'utiliser np.diff(np.angle(x)) mais
     * np.angle() sort un résultat dans [-pi, pi]. Et donc quand p(t) augmente ça fait des sauts de phase...
     *
     * Du coup la solution: multiplier x[n] par le conjugué de x[n-1].
     * Démonstration:
     *
     *     z[n] = x[n] * conj(x[n-1])
     *          = A[n]*exp(j*p[n])   *    A[n-1]*exp(-j*p[n-1])
     *          = A[n]*A[n-1]    *    exp(j(p[n] - p[n-1]))
     *
     *     Or le produit A[n]*A[n-1] est un reel positif pour tout n donc il n'impacte pas la phase.
     *     Par conséquent l'argument de z est directmeent p[n]-p[n-1].
     *
     * Et par conséquent, f[n] = fe/(2*pi) * arg(z[n])
     */

    // 1) Calculating x[n] * conj(x[n-1])
    // reminder: Z = Z1 x Z2 <=> (a+jb) x (c+jd) <=> (ac-bd) + j(ad+bc)
    double *z = malloc(len * sizeof(double));
    for (size_t n = 2; n + 1 < len; n += 2)
    {
        double a =  iq[n];
        double b =  iq[n + 1];
        double c =  iq[n - 2];
        double d = -iq[n - 1];
        z[n]     = a * c - b * d;
        z[n+1]   = a * d + b * c;
    }

    // 2) Computing the instantaneous frequency from the phase increment of Z
    double mul = SAMPLE_RATE / (2.0 * M_PI);
    double *freq = malloc((len/2) * sizeof(double));
    for (size_t n = 1; n < len/2; n++)
    {
        freq[n] = atan(z[2*n] / z[2*n + 1]) * mul;
    }
    freq[0] = freq[1]; // pad one sample

    return freq;
}