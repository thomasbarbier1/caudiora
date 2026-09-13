#include <stdio.h>
#include <semaphore.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "processing.h"

#include <assert.h>

#include "acquisition.h"
#include "radio_interface.h"
#include "output.h"

#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define TESTNAME "caudiora_test.wav"

#define AUDIO_RECORDING_SIZE (size_t) 16777216
#define LP_FILTER_CUTOFF     (double) 125000
#define MIN_SPAN_HZ          (double) 10
#define AMPLITUDE_FLOOR      (double) 1e-8
#define TWOPI                (double) (2.0 * M_PI)
#define OUTPUT_AMPLITUDE     (double) 0.9
#define AUDIO_SAMPLE_PERIOD  (double) (1 / AUDIO_SAMPLE_RATE)
#define STRETCH_FACTOR       (double) 4.0
#define ENVELOPE_TAU_S       (double) 500e-6   /* constante de temps du lissage */
#define SQUELCH_RATIO        (double) 0.25     /* seuil de gate, en fraction du max */
#define FILTER_COEFF_NB      (int) 79
#define DOWNSAMPING_FS       (double) 240000

// see filter_coeff.py python script to see how the coeff are pre-calculated
static const double FILTER_COEFFS[FILTER_COEFF_NB] =
{
    9.40272334e-05,  6.74913599e-05,  3.82907759e-05,  7.45567487e-06,
   -2.23698387e-05, -4.62285855e-05, -5.69030291e-05, -4.60004303e-05,
   -6.14087327e-06,  6.59539307e-05,  1.65959933e-04,  2.78770172e-04,
    3.76508342e-04,  4.18604963e-04,  3.54626272e-04,  1.30198008e-04,
   -3.04116971e-04, -9.81683840e-04, -1.90764386e-03, -3.04742351e-03,
   -4.31782752e-03, -5.58262237e-03, -6.65420583e-03, -7.30236137e-03,
   -7.27026729e-03, -6.29696472e-03, -4.14451002e-03, -6.27186109e-04,
    4.36044648e-03,  1.08201047e-02,  1.86341904e-02,  2.75606595e-02,
    3.72399438e-02,  4.72142235e-02,  5.69579301e-02,  6.59170016e-02,
    7.35532733e-02,  7.93896455e-02,  8.30514092e-02,  8.42993952e-02,
    8.30514092e-02,  7.93896455e-02,  7.35532733e-02,  6.59170016e-02,
    5.69579301e-02,  4.72142235e-02,  3.72399438e-02,  2.75606595e-02,
    1.86341904e-02,  1.08201047e-02,  4.36044648e-03, -6.27186109e-04,
   -4.14451002e-03, -6.29696472e-03, -7.27026729e-03, -7.30236137e-03,
   -6.65420583e-03, -5.58262237e-03, -4.31782752e-03, -3.04742351e-03,
   -1.90764386e-03, -9.81683840e-04, -3.04116971e-04,  1.30198008e-04,
    3.54626272e-04,  4.18604963e-04,  3.76508342e-04,  2.78770172e-04,
    1.65959933e-04,  6.59539307e-05, -6.14087327e-06, -4.60004303e-05,
   -5.69030291e-05, -4.62285855e-05, -2.23698387e-05,  7.45567487e-06,
    3.82907759e-05,  6.74913599e-05,  9.40272334e-05
};

typedef struct
{
    // private
    atomic_bool running;
    double      amplitude_floor;
    double      filter_fc;
    double      audio_span_hz;
    double      audio_center_hz;
    double      input_bw;
    double      input_fs;
    double      downsampled_fs;
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
static double* decimate(const double *input_signal, size_t input_length, double fs_in, double fs_out, size_t *output_len);
static double* zero_centering_and_double_conversion(const uint8_t *iq, size_t len);
static int compute_instantaneous_frequency(const double *iq, size_t iq_len, double **freq, double **amp, size_t *freq_len);
static void prepare_amplitude_envelope(double *amp, size_t len);
static void map_to_audio_band(double *freq, size_t N);
static double* time_stretch(double *x_signal, size_t x_len, double stretch_factor, size_t* y_len);
static double* linear_interpolation(const double* x_time, const double* x_signal, size_t x_len, const double* y_time, size_t y_len);
static void interpolate_value(const double* x_signal, const double* x_vect, size_t left_idx, size_t right_idx, double* y_signal, const double* y_vect, size_t y_idx);
static void audio_render(const double* f, double* y, size_t len, double *amp_envelope);
static double clip(double val, double lower, double upper);

/***********************************************************************************************************************
 * Public functions implementation
 **********************************************************************************************************************/

int processing_init(void)
{
    dsp_data.iq              = get_acquisition_buffer();
    dsp_data.processing_sem  = get_acquisition_semaphore();
    dsp_data.filter_fc       = LP_FILTER_CUTOFF;
    dsp_data.amplitude_floor = AMPLITUDE_FLOOR;
    dsp_data.input_bw        = (double) BANDWIDTH;
    dsp_data.input_fs        = (double) SAMPLE_RATE;
    dsp_data.downsampled_fs  = DOWNSAMPING_FS;
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
            int rc = processing_pipeline(dsp_data.iq, dsp_data.len);
            if (rc != 0)
            {
                printf("failed to process signal.\n");
            }
        }
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

    // Zero centering and cast to double
    double* iq_as_double = zero_centering_and_double_conversion(iq, len);
    if (iq_as_double == NULL) return 1;

    // Decimation to 240 kHz
    size_t decimated_len;
    double* decimated = decimate(iq_as_double, len, dsp_data.input_fs, dsp_data.downsampled_fs, &decimated_len);
    if (decimated == NULL) return 1;

    // Recovering the instantaneous frequency from the phase
    size_t  freq_len = 0;
    double* freq     = NULL;
    double* amp      = NULL;
    compute_instantaneous_frequency(decimated, decimated_len, &freq, &amp, &freq_len);
    prepare_amplitude_envelope(amp, freq_len);

    // Mapping to audioband
    map_to_audio_band(freq, freq_len);

    // Time stretching
    size_t stretched_len;
    double* stretched_freq = time_stretch(freq, freq_len, dsp_data.stretch_factor, &stretched_len);
    double* stretched_amp  = time_stretch(amp, freq_len, dsp_data.stretch_factor, &stretched_len);

    // Generating the audio signal from the frequency
    double* audio  = malloc(stretched_len * sizeof(double));
    audio_render(stretched_freq, audio, stretched_len, stretched_amp);

    // Exporting to wav
    write_wav(NULL, audio, stretched_len);

    free(iq_as_double);
    free(decimated);
    free(freq);
    free(amp);
    free(stretched_freq);
    free(stretched_amp);
    free(audio);

    return 0;
}

static double* zero_centering_and_double_conversion(const uint8_t *iq, size_t len)
{
    double *iq_as_double = malloc(len * sizeof(double));
    if (iq_as_double == NULL)
    {
        return NULL;
    }

    for (size_t i = 0; i + 1 < len; i+=2)
    {
        iq_as_double[i]     = (double) iq[i]     - 127.5;
        iq_as_double[i + 1] = (double) iq[i + 1] - 127.5;
    }
    return iq_as_double;
}

/**
 *  Anti-aliasing filter + decimation over an interleaved IQ signal
 *  fs_in  = 2400000
 *  fs_out = 240000
 *  M      = 10
 *
 *  The input signal uses +/- 62.5 kHz, so f_nyquist = 125kHz
 *  we take roughly x2 for the margin, so 250 kHz, and we lower to 240kHz so that we have a round 10 factor
 *  between f_in and f_out
 *  The filter does not filter until f = 62.5kHz and attenuate -60dB when > 175 kHz
 *
 *   |H(f)| dB
 *    0 |---------\
 *      |          \
 *  -60 |           \_______________
 *      |___________________________> f(Hz)
 *      0      62.5  175
 *
 *      This filter introduces a delay in (FILTER_COEFF_NB - 1) / 2 complex samples
 *
 */
static double* decimate(const double *input_signal, size_t input_length, double fs_in, double fs_out, size_t *output_len)
{
    if (input_signal == NULL || output_len == NULL || fs_out <= 0 || fs_in < fs_out * 2)
    {
        return NULL;
    }

    *output_len = 0;

    if (input_length % 2 != 0)
    {
        fprintf(stderr, "decimation: interleaved IQ buffer must have an even length\n");
        return NULL;
    }

    const size_t nb_samples = input_length / 2;
    const size_t nb_coeff   = FILTER_COEFF_NB;

    if (nb_samples < nb_coeff)
    {
        fprintf(stderr, "decimation: signal too short for a single complete window\n");
        return NULL;
    }

    const double ratio = fs_in / fs_out;
    const size_t decim_factor = (size_t) llround(ratio);

    if (fabs(ratio - (double) decim_factor) > 1e-9 || decim_factor == 0)
    {
        fprintf(stderr, "decimation: fs_in / fs_out must be an integer (got %f)\n", ratio);
        return NULL;
    }

    const size_t output_samples = (nb_samples - nb_coeff) / decim_factor + 1;
    double* output_signal       = malloc(output_samples * 2 * sizeof(double));
    if (output_signal == NULL)
    {
        return NULL;
    }

    size_t out_idx = 0;

    for (size_t n = nb_coeff - 1; n < nb_samples; n += decim_factor)
    {
        double sum_i = 0;
        double sum_q = 0;

        for (size_t k = 0; k < nb_coeff; k++)
        {
            sum_i += FILTER_COEFFS[k] * input_signal[2 * (n - k)];
            sum_q += FILTER_COEFFS[k] * input_signal[2 * (n - k) + 1];
        }

        output_signal[2 * out_idx]     = sum_i;
        output_signal[2 * out_idx + 1] = sum_q;
        out_idx++;
    }

    assert(out_idx == output_samples);

    *output_len = output_samples * 2;
    return output_signal;
}

static int compute_instantaneous_frequency(const double *iq, const size_t iq_len, double **freq, double **amp, size_t *freq_len)
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

    if (iq == NULL || iq_len % 2 != 0 || iq_len < 4)
    {
        return -1;
    }

    const size_t f_len = iq_len / 2;
    double* frequency = malloc(f_len * sizeof(double));
    double* amplitude = malloc(f_len * sizeof(double));
    if (frequency == NULL || amplitude == NULL) return -1;
    const double mul = dsp_data.downsampled_fs / TWOPI;

    // z[n] * conj(z[n-1]) = (ac + bd) + j(bc - ad)
    for (size_t n = 1; n < f_len; n++)
    {
        const double a = iq[2*n];
        const double b = iq[2*n + 1];
        const double c = iq[2*n - 2];
        const double d = iq[2*n - 1];

        const double re = a * c + b * d;
        const double im = b * c - a * d;
        frequency[n] = mul * atan2(im, re);
        amplitude[n] = sqrt(a * a + b * b);
    }

    // pad one sample
    frequency[0] = frequency[1];
    amplitude[0]  = amplitude[1];

    *freq_len = f_len;
    *freq = frequency;
    *amp = amplitude;

    return 0;
}

static void prepare_amplitude_envelope(double *amp, size_t len)
{
    if (amp == NULL || len < 2) return;

    /* low-pass 1st order, go and back to cancel delay */
    const double alpha = 1.0 - exp(-1.0 / (ENVELOPE_TAU_S * dsp_data.downsampled_fs));
    double acc = amp[0];
    for (size_t i = 0; i < len; i++)      { acc += alpha * (amp[i] - acc); amp[i] = acc; }
    for (size_t i = len; i-- > 0; )       { acc += alpha * (amp[i] - acc); amp[i] = acc; }

    /* normalization from the maximum */
    double max = 0.0;
    for (size_t i = 0; i < len; i++) max = MAX(max, amp[i]);

    if (max < dsp_data.amplitude_floor)
    {
        memset(amp, 0, len * sizeof(double));
        return;
    }

    /* gate + scaling */
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

    const double x_duration  = (double) x_len / dsp_data.downsampled_fs;
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
        x_time[i] = (double) i / dsp_data.downsampled_fs;
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
        // y2[i] = OUTPUT_AMPLITUDE * sin(phase);
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

