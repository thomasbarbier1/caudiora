#include <stdio.h>
#include <semaphore.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "processing.h"

#include <string.h>

#include "acquisition.h"
#include "radio_interface.h"
#include "output.h"

#define AUDIO_RECORDING_SIZE (size_t) 16777216
#define MIN_SPAN_HZ          (double) 10
#define AMPLITUDE_FLOOR      (double) 1e-8
#define TWOPI                (double) (2.0 * M_PI)
#define OUTPUT_AMPLITUDE     (double) 0.9
#define AUDIO_SAMPLE_PERIOD  (double) (1 / AUDIO_SAMPLE_RATE)
#define STRETCH_FACTOR       (double) 4.0
#define ENVELOPE_TAU_S       (double) 500e-6   /* constante de temps du lissage */
#define SQUELCH_RATIO        (double) 0.25     /* seuil de gate, en fraction du max */
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define TESTNAME "caudiora_test.wav"

typedef struct
{
    // private
    atomic_bool running;
    double      amplitude_floor;
    double      audio_span_hz;
    double      audio_center_hz;
    double      input_bw;
    double      input_fs;
    double      output_fs;
    double      stretch_factor;

    // public
    uint8_t  *iq;
    uint32_t len;
    sem_t*   processing_sem;

} processing_t;

static processing_t dsp_data;

/***********************************************************************************************************************
 * Private functions declaration
 **********************************************************************************************************************/

// dsp pipeline functions
static int processing_pipeline(const uint8_t *iq, size_t len);
static double* zero_centering_and_double_conversion(const uint8_t *iq, size_t len);
static void normalize_complex_amplitude(double *iq, size_t len);
static double* compute_instantaneous_frequency(const double *iq, size_t iq_len, double *freq, double *amp, size_t freq_len);
static void prepare_amplitude_envelope(double *amp, size_t len);
static void map_to_audio_band(double *freq, size_t N);
static double* time_stretch(double *x_signal, size_t x_len, double stretch_factor, size_t* y_len);
static double* linear_interpolation(const double* x_time, const double* x_signal, size_t x_len, const double* y_time, size_t y_len);
static void interpolate_value(const double* x_signal, const double* x_vect, size_t left_idx, size_t right_idx, double* y_signal, const double* y_vect, size_t y_idx);
static void audio_render(const double* f, double* y, size_t len, double *amp_envelope);
// helpers
static double clip(double val, double lower, double upper);
int export_signal_csv_cmplx(const char* filename, const uint8_t *data, size_t length, size_t offset);
int export_signal_csv(const char* filename, const double *data, size_t length, size_t offset);
/***********************************************************************************************************************
 * Public functions implementation
 **********************************************************************************************************************/

int processing_init(void)
{
    dsp_data.iq              = get_acquisition_buffer();
    dsp_data.processing_sem  = get_acquisition_semaphore();
    dsp_data.amplitude_floor = AMPLITUDE_FLOOR;
    dsp_data.input_bw        = (double) BANDWIDTH;
    dsp_data.input_fs        = (double) SAMPLE_RATE;
    dsp_data.output_fs       = (double) AUDIO_SAMPLE_RATE;
    dsp_data.audio_span_hz   = (double) 1000.0;
    dsp_data.audio_center_hz = (double) 2000.0;
    dsp_data.stretch_factor  = STRETCH_FACTOR;

    if (dsp_data.iq == NULL)
    {
        return 1;
    }

    init_output();
    dsp_data.running = true;
    return 0;
}

int processing_stop(void)
{
    dsp_data.running = false;
    sem_post(dsp_data.processing_sem); // in case the semaphore is waiting, we unlock it first
    sem_destroy(dsp_data.processing_sem);

    return 0;
}

void* processing_start(void *ctx)
{
    while (dsp_data.running)
    {
        sem_wait(dsp_data.processing_sem);
        get_buffer_length(&dsp_data.len);
        if (dsp_data.len != 0)
        {
            // double v = mean_power(processing_data.iq, processing_data.len);
            // printf("v: %lf \t len: %u\n", v, processing_data.len);
            processing_pipeline(dsp_data.iq, dsp_data.len);
        }
        unpause_acquisition();
    }

    return NULL;
}

void test()
{
    size_t len = 1000000;
    double* x = malloc(len * sizeof(double));
    for (int i=0; i<len; i++)
    {
        x[i] = (double) 0.3 * sin(TWOPI * 200 * i / AUDIO_SAMPLE_RATE);
    }

    write_wav(TESTNAME, x, len);
    free(x);
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

    // to be implemented: remove the DC offset and apply a LPF and decimate to 400kHz

    // Getting the instantaneous frequency from the phase
    size_t freq_len = len / 2;
    double* freq = malloc(freq_len * sizeof(double));
    double* amp  = malloc(freq_len * sizeof(double));
    compute_instantaneous_frequency(iq_as_double, len, freq, amp, freq_len);
    prepare_amplitude_envelope(amp, freq_len);

    // Mapping to audioband
    map_to_audio_band(freq, freq_len);

    // Time stretching
    size_t stretched_len;
    double* stretched_freq = time_stretch(freq, freq_len, dsp_data.stretch_factor, &stretched_len);
    double* stretched_amp  = time_stretch(amp, freq_len, dsp_data.stretch_factor, &stretched_len);

    // Generating the audio signal from the frequency
    double* audio = malloc(stretched_len * sizeof(double));
    audio_render(stretched_freq, audio, stretched_len, stretched_amp);

    // Exporting to wav
    write_wav(NULL, audio, (size_t) stretched_len);

    free(iq_as_double);
    free(freq);
    free(amp);
    free(stretched_freq);
    free(stretched_amp);
    free(audio);

    return 0;
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
    if (max_magnitude < dsp_data.amplitude_floor || max_magnitude == 0.0)
    {
        return;
    }

    for (size_t i = 0; i < len; i++)
    {
        iq[i] /= max_magnitude;
    }
}

static double *compute_instantaneous_frequency(const double *iq, const size_t iq_len, double *freq, double *amp, const size_t freq_len)
{
    /* Estimate instantaneous frequency from the phase increment. */

    /*
     * Dans cette fonction on cherche à obtenir f[n], la frequence instannee.
     * Or f(t) = 1/(2*pi) * dp(t)/dt   (avec p(t) la phase instantanée)
     * On peut approximer ça en discret par:
     *     f[n] =   1/(2*pi) * (p[n]-p[n-1])/Te
     *          =  fe/(2*pi) * (p[n]-p[n-1])
     *
     * Il nous faut donc récupérer p[n]-p[n-1]. La solution est de multiplier x[n] par le conjugué de x[n-1]:
     *     z[n] = x[n] * conj(x[n-1])
     *          = A[n]*exp(j*p[n])   *    A[n-1]*exp(-j*p[n-1])
     *          = A[n]*A[n-1]    *    exp(j(p[n] - p[n-1]))
     *
     * Le module A[n] * A[n-1] est un reel positif pour tout n donc il n'impacte pas l'argument.
     * Donc arg(z[n]) est directement p[n] - p[n-1].
     * Conclusion: f[n] = fe/(2*pi) * arg(z[n])
     */

    if (iq == NULL || freq == NULL || amp == NULL || iq_len % 2 != 0 || iq_len < 4)
    {
        return NULL;
    }

    const double mul = dsp_data.input_fs / TWOPI;

    // z[n] * conj(z[n-1]) = (ac + bd) + j(bc - ad)
    for (size_t n = 1; n < freq_len; n++)
    {
        const double a = iq[2*n];
        const double b = iq[2*n + 1];
        const double c = iq[2*n - 2];
        const double d = iq[2*n - 1];

        const double re = a * c + b * d;
        const double im = b * c - a * d;
        freq[n] = mul * atan2(im, re);
        amp[n]  = sqrt(a * a + b * b);
    }

    // pad one sample
    freq[0] = freq[1];
    amp[0]  = amp[1];

    return freq;
}

static void prepare_amplitude_envelope(double *amp, size_t len)
{
    if (amp == NULL || len < 2) return;

    /* 1. Lissage passe-bas 1er ordre, aller puis retour pour annuler le retard */
    const double alpha = 1.0 - exp(-1.0 / (ENVELOPE_TAU_S * dsp_data.input_fs));
    double acc = amp[0];
    for (size_t i = 0; i < len; i++)      { acc += alpha * (amp[i] - acc); amp[i] = acc; }
    for (size_t i = len; i-- > 0; )       { acc += alpha * (amp[i] - acc); amp[i] = acc; }

    /* 2. Normalisation par le maximum */
    double max = 0.0;
    for (size_t i = 0; i < len; i++) max = MAX(max, amp[i]);

    if (max < dsp_data.amplitude_floor)
    {
        memset(amp, 0, len * sizeof(double));
        return;
    }

    /* 3. Gate + mise à l'échelle finale */
    for (size_t i = 0; i < len; i++)
    {
        double g = (amp[i] / max - SQUELCH_RATIO) / (1.0 - SQUELCH_RATIO);
        amp[i] = (g > 0.0) ? OUTPUT_AMPLITUDE * g : 0.0;
    }
}

static void map_to_audio_band(double *freq, const size_t N)
{
    const double nyq   = 0.5 * dsp_data.output_fs;
    const double upper = 0.92 * nyq;
    const double lower = 50.0;

    const double src_half_span = 0.5 * dsp_data.input_bw;
    const double scale         = dsp_data.audio_span_hz / MAX(src_half_span, MIN_SPAN_HZ);

    for (size_t i = 0; i < N; i++)
    {
        double val = scale * freq[i] + dsp_data.audio_center_hz;
        val = clip(val, lower, upper);
        freq[i] = val;
    }
}

static double* time_stretch(double *x_signal, size_t x_len, double stretch_factor, size_t* y_len)
{
    if (x_signal == NULL || y_len == NULL || x_len < 2)
    {
        if (y_len != NULL) *y_len = 0;
        return NULL;
    }

    *y_len = 0;

    if (stretch_factor <= 1.0)
    {
        double *copy = malloc(x_len * sizeof(double));
        if (copy == NULL)
        {
            return NULL;
        }
        memcpy(copy, x_signal, x_len * sizeof(double));
        *y_len = x_len;
        return copy;
    }

    const double x_duration  = (double) x_len / dsp_data.input_fs;
    const double y_duration  = x_duration * stretch_factor;
    double n_exact = round(y_duration * dsp_data.output_fs);
    if (n_exact < 2.0)
    {
        n_exact = 2.0;
    }
    const size_t n_len = (size_t) n_exact;

    double *x_time = malloc(x_len * sizeof(double));
    double *y_time = malloc(n_len * sizeof(double));
    if (x_time == NULL || y_time == NULL)
    {
        free(x_time);
        free(y_time);
        return NULL;
    }

    /* Making the time axis for the input signal, x */
    for (size_t i = 0; i < x_len; i++)
    {
        x_time[i] = (double) i / dsp_data.input_fs;
    }

    /* Making the time axis for the output signal, y */
    for (size_t i = 0; i < n_len; i++)
    {
        y_time[i] = (double) i / dsp_data.output_fs / stretch_factor;
    }

    double *stretched = linear_interpolation(x_time, x_signal, x_len, y_time, n_len);

    free(x_time);
    free(y_time);

    if (stretched == NULL)
    {
        return NULL;
    }

    *y_len = n_len;
    return stretched;
}

static double* linear_interpolation(const double* x_time, const double* x_signal, const size_t x_len, const double* y_time, const size_t y_len)
{
    if (x_time == NULL || x_signal == NULL || y_time == NULL || x_len < 2 || y_len == 0)
    {
        return NULL;
    }

    double* y_signal = malloc(y_len * sizeof(double));
    if (y_signal == NULL)
    {
        return NULL;
    }

    size_t yi = 0;

    // Left-side extrapolation: points that are before x_time[0]
    while (yi < y_len && y_time[yi] < x_time[0])
    {
        interpolate_value(x_signal, x_time, 0, 1, y_signal, y_time, yi);
        yi++;
    }

    if (yi >= y_len) return y_signal;

    // Interpolation: points that are in range [xi, xi+1[
    for (size_t xi = 0; xi + 1 < x_len; xi++)
    {
        while (yi < y_len && y_time[yi] >= x_time[xi] && y_time[yi] < x_time[xi+1])
        {
            interpolate_value(x_signal, x_time, xi, xi+1, y_signal, y_time, yi);
            yi++;
        }
    }

    // Right-side extrapolation: points that are after x[x_len - 1]
    while (yi < y_len)
    {
        interpolate_value(x_signal, x_time, x_len - 2, x_len - 1, y_signal, y_time, yi);
        yi++;
    }

    return y_signal;
}

static void interpolate_value(const double* x_signal, const double* x_vect, size_t left_idx, size_t right_idx, double* y_signal, const double* y_vect, const size_t y_idx)
{
    const double dx    = x_vect[right_idx] - x_vect[left_idx];
    const double slope = (dx != 0.0) ? (x_signal[right_idx] - x_signal[left_idx]) / dx : 0.0;
    y_signal[y_idx] = x_signal[left_idx] + slope * (y_vect[y_idx] - x_vect[left_idx]);
}

static void audio_render(const double* f, double* y, const size_t len, double *amp_envelope)
{
    if (f == NULL || y == NULL || amp_envelope == NULL || len < 2) return;

    const double sample_period = 1 / dsp_data.output_fs;
    const double nyquist = 0.5 / sample_period;
    double phase = 0.0;

    for (size_t i = 0; i < len; i++)
    {
        y[i] = clip(amp_envelope[i], -1, 1) * sin(phase);
        // y[i] = OUTPUT_AMPLITUDE * sin(phase);
        // integration of frequency (trapezoid)
        double fi = f[i];
        double fp = (i > 0) ? f[i - 1] : f[0];
        double favg = 0.5 * (fi + fp);

        if (favg < 0.0) favg = 0.0;
        if (favg > nyquist) favg = nyquist;   // prevent aliasing

        phase += TWOPI * favg * sample_period;
        if (phase >= TWOPI) phase -= TWOPI;
    }
}

static double clip(double val, double lower, double upper)
{
    // Clip val between lower and upper bounds
    val = val < lower ? lower : val;
    val = val > upper ? upper : val;
    return val;
}

int export_signal_csv_cmplx(const char* filename, const uint8_t *data, size_t length, size_t offset)
{
    FILE *f = fopen(filename, "w");
    if (!f) {
        perror("fopen signal.csv");
        return -1;
    }

    fprintf(f, "index,value\n");
    for (size_t i = offset; i < offset + length; i++) {
        fprintf(f, "%zu,%u\n", i, data[i]);
    }

    if (fclose(f) != 0) {
        perror("fclose signal.csv");
        return -1;
    }
    return 0;
}

int export_signal_csv(const char* filename, const double *data, size_t length, size_t offset)
{
    FILE *f = fopen(filename, "w");
    if (!f) {
        perror("fopen signal.csv");
        return -1;
    }

    fprintf(f, "index,value\n");
    for (size_t i = offset; i < offset+length; i++) {
        fprintf(f, "%zu,%.9g\n", i, data[i]);
    }

    if (fclose(f) != 0) {
        perror("fclose signal.csv");
        return -1;
    }
    return 0;
}

