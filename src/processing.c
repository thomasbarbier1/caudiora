/*
 * Copyright (c) 2018, Thomas Barbier
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/**
 * @file processing.c
 * @brief Receives the recording, and process the signal to make it audible. Then, output the rendered audio signal.
 */

#include <stdio.h>
#include <semaphore.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include "processing.h"
#include "acquisition.h"
#include "radio_interface.h"
#include "output.h"

#define AUDIO_RECORDING_SIZE (size_t) 16777216
#define LP_FILTER_CUTOFF     (double) 125000
#define MIN_SPAN_HZ          (double) 10
#define AMPLITUDE_FLOOR      (double) 1e-8
#define TWOPI                (double) (2.0 * M_PI)
#define OUTPUT_AMPLITUDE     (double) 0.9
#define AUDIO_SAMPLE_PERIOD  (double) (1 / AUDIO_SAMPLE_RATE)
#define STRETCH_FACTOR       (double) 4.0
#define ENVELOPE_TAU_S       (double) 500e-6   /* time constant for smoothing */
#define SQUELCH_RATIO        (double) 0.25     /* gate threshold as a fraction of max */
#define FILTER_COEFF_NB      (int) 79
#define DOWNSAMPING_FS       (double) 240000
#define PIPELINE_FAIL_CODE   (int) 1
#define PIPELINE_SUCCES_CODE (int) 0

#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))

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
    double      zero_centering_offset;

    // public
    uint8_t  *iq;
    uint32_t len;
    sem_t*   processing_sem;

} processing_t;

static processing_t dsp_data;

/***********************************************************************************************************************
 * Private functions declaration
 **********************************************************************************************************************/

static int processing_pipeline(const uint8_t *iq, size_t iq_len);
static void decimate(const uint8_t *input_signal, size_t input_length, double **output_signal, size_t *output_length, double fs_in, double fs_out);
static void compute_instantaneous_frequency(const double *iq, size_t iq_len, double **freq, double **amp, size_t *freq_len);
static void prepare_amplitude_envelope(double *amp, size_t amp_len);
static void map_to_audio_band(double *freq, size_t N);
static void time_stretch(const double *source_signal, size_t source_length, double **destination_signal, size_t *destination_length, double stretch_factor);
static double* linear_interpolation(const double* source_time, const double* source_signal, size_t source_length, const double* destination_time, size_t destination_length);
static void interpolate_value(const double* source_time, const double* source_signal, const double* destination_time, double* destination_signal, size_t left_idx, size_t right_idx, size_t destination_idx);
static void audio_render(const double* freq_t, size_t freq_len, const double *amp_envelope, double **output_signal);
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

    if (dsp_data.stretch_factor <= 1.0)
    {
        fprintf(stderr, "Stretching factor must be > 1.0\n");
        return 1;
    }

    if (dsp_data.iq == NULL)
    {
        return 1;
    }

    /* Pre-computing zero_centering_offset (see decimate() function) */
    dsp_data.zero_centering_offset = 0;
    for (size_t i = 0; i < FILTER_COEFF_NB; i++)
    {
        dsp_data.zero_centering_offset += FILTER_COEFFS[i];
    }
    dsp_data.zero_centering_offset *= 127.5;

    init_wav_export();
    
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

void* processing_thread(void *ctx)
{
    while (dsp_data.running)
    {
        sem_wait(dsp_data.processing_sem);
        get_buffer_length(&dsp_data.len);
        if (dsp_data.len != 0)
        {
            const int rc = processing_pipeline(dsp_data.iq, dsp_data.len);
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

/**
 *  @brief Process the IQ samples (8-bit, interleaved and centered on 127.5)
 *  Steps:
 *      1. Anti-aliasing filter and signal decimation
 *      2. Getting the instantaneous frequency from the phase increment
 *      3. Frequency mapping to the audio band
 *      4. Time stretching on the instantaneous frequency
 *      5. Output signal final computation from the instantaneous frequency and amplitude
 *      6. Final audio file rendering (.wav)
 *
 *  @param iq  (in) Array of complex IQ samples (8-bits, interleaved, centered on 127.5)
 *  @param iq_len (in) length if the IQ samples
 *  @return error_code: PIPELINE_FAIL_CODE is fail, else PIPELINE_SUCCESS_CODE
 */
static int processing_pipeline(const uint8_t *iq, const size_t iq_len)
{
    if (iq == NULL || iq_len < 2 || iq_len % 2 != 0)
    {
        return 1;
    }

    int rc = PIPELINE_FAIL_CODE;

    double* decimated      = NULL;
    double* freq           = NULL;
    double* amp            = NULL;
    double* stretched_freq = NULL;
    double* stretched_amp  = NULL;
    double* audio          = NULL;

    size_t decimated_len   = 0;
    size_t freq_len        = 0;
    size_t stretched_len   = 0;

    /* Decimation: 2.4 MHz -> 240 kHz ------------------------------------------------------------------------------- */
    decimate(iq, iq_len, &decimated, &decimated_len, dsp_data.input_fs, dsp_data.downsampled_fs);
    if (decimated == NULL || decimated_len == 0) goto exit;

    /* Recovering the instantaneous frequency from the phase increment ---------------------------------------------- */
    compute_instantaneous_frequency(decimated, decimated_len, &freq, &amp, &freq_len);
    prepare_amplitude_envelope(amp, freq_len);
    if (freq == NULL || amp == NULL || freq_len == 0) goto exit;

    /* Mapping to audioband ----------------------------------------------------------------------------------------- */
    map_to_audio_band(freq, freq_len);

    /* Time stretching ---------------------------------------------------------------------------------------------- */
    time_stretch(freq, freq_len, &stretched_freq, &stretched_len, dsp_data.stretch_factor);
    time_stretch(amp, freq_len, &stretched_amp, &stretched_len, dsp_data.stretch_factor);
    if (stretched_freq == NULL || stretched_amp == NULL || stretched_len == 0) goto exit;

    /* Generating the audio signal from the frequency --------------------------------------------------------------- */
    audio_render(stretched_freq, stretched_len, stretched_amp, &audio);
    if (audio == NULL) goto exit;

    /* Exporting to wav --------------------------------------------------------------------------------------------- */
    write_wav(NULL, audio, stretched_len);

    /* Audio stream ------------------------------------------------------------------------------------------------- */
    output_play(audio, stretched_len);

    rc = PIPELINE_SUCCES_CODE;

exit:
    free(decimated);
    free(freq);
    free(amp);
    free(stretched_freq);
    free(stretched_amp);
    free(audio);

    return rc;
}

/**
 *  @brief:
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
 *  As the input IQ samples are centered in 127.5, this function also applies a centering in zero:
 *  In the discrete world, the convolution product can be written like so:
 *  Σ cₖ·xₙ₋ₖ
 *  But as we want to center in zero, we will perform the following:
 *  Σ cₖ·(xₙ₋ₖ − 127,5)
 *  This expression can be re-written like so:
 *  Σ cₖ·xₙ₋ₖ − 127,5 · Σ cₖ
 *  This allows to negate a constant at the end of the computation, instead of substracting 127.5 n-times.
 *
 *  @param [in]  input_signal  the signal to decimate
 *  @param [in]  input_length  the length of the input signal
 *  @param [out] output_signal the output signal
 *  @param [out] output_length length of the output signal
 *  @param [in]  fs_in         sampling frequency of the input signal
 *  @param [in]  fs_out        sampling frequency of the output signal
 */
static void decimate(const uint8_t *input_signal, const size_t input_length, double **output_signal, size_t *output_length, const double fs_in, const double fs_out)
{
    if (input_signal == NULL || output_length == NULL || output_signal == NULL)
    {
        fprintf(stderr, "[processing] decimate: received NULL pointer\n");
        return;
    }

    if (fs_out <= 0 || fs_in < fs_out * 2)
    {
        fprintf(stderr, "[processing] decimate: invalid fs_in or fs_out\n");
        return;
    }

    if (input_length % 2 != 0)
    {
        fprintf(stderr, "[processing] decimation: interleaved IQ buffer must have an even length\n");
        return;
    }

    const size_t nb_samples = input_length / 2;
    const size_t nb_coeff   = FILTER_COEFF_NB;

    if (nb_samples < nb_coeff)
    {
        fprintf(stderr, "[processing] decimation: signal too short for a single complete window\n");
        return;
    }

    const double ratio = fs_in / fs_out;
    const size_t decim_factor = (size_t) llround(ratio);

    if (fabs(ratio - (double) decim_factor) > 1e-9 || decim_factor == 0)
    {
        fprintf(stderr, "[processing] decimation: fs_in / fs_out must be an integer (got %f)\n", ratio);
        return;
    }

    const size_t output_samples = (nb_samples - nb_coeff) / decim_factor + 1;
    double* out                 = malloc(output_samples * 2 * sizeof(double));
    if (out == NULL)
    {
        return;
    }

    size_t out_idx = 0;
    for (size_t n = nb_coeff - 1; n < nb_samples; n += decim_factor)
    {
        double sum_i = 0;
        double sum_q = 0;

        for (size_t k = 0; k < nb_coeff; k++)
        {
            sum_i += FILTER_COEFFS[k] * (double) input_signal[2 * (n - k)];
            sum_q += FILTER_COEFFS[k] * (double) input_signal[2 * (n - k) + 1];
        }

        out[2 * out_idx]     = sum_i - dsp_data.zero_centering_offset;
        out[2 * out_idx + 1] = sum_q - dsp_data.zero_centering_offset;
        out_idx++;
    }

    assert(out_idx == output_samples);
    *output_length = output_samples * 2;
    *output_signal = out;
}

/**
 *
 * @param [in]  iq       the input IQ signal
 * @param [in]  iq_len   length of the IQ signal
 * @param [out] freq     the instantaneous frequency
 * @param [out] amp      the instantenous amplitude
 * @param [out] freq_len the length of the frequency/amplitude arrays
 */
static void compute_instantaneous_frequency(const double *iq, const size_t iq_len, double **freq, double **amp,
                                            size_t *freq_len)
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

    if (iq == NULL)
    {
        fprintf(stderr, "[processing] compute_instanteous_frequency: received NULL pointer.\n");
        return;
    }

    if (iq_len % 2 != 0 || iq_len < 4)
    {
        fprintf(stderr, "processing] compute_instanteous_frequency: received invalid iq_len\n");
        return;
    }

    const size_t f_len = iq_len / 2;
    double* frequency = malloc(f_len * sizeof(double));
    double* amplitude = malloc(f_len * sizeof(double));
    if (frequency == NULL || amplitude == NULL)
    {
        free(frequency);
        free(amplitude);
        return;
    }
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
}

/**
 * @brief applies a low-pass filter and normalize the amplitude envelope
 * @param [in/out] amp amplitude envelope of the signal
 * @param [in]     amp_len length of the envelope
 */
static void prepare_amplitude_envelope(double *amp, const size_t amp_len)
{
    if (amp == NULL || amp_len < 2)
    {
        fprintf(stderr, "processing] prepare_amplitude_envelope: received NULL pointer or invalid amp_len\n");
        return;
    }

    /* low-pass 1st order, go and back to cancel delay */
    const double alpha = 1.0 - exp(-1.0 / (ENVELOPE_TAU_S * dsp_data.downsampled_fs));
    double acc = amp[0];
    for (size_t i = 0; i < amp_len; i++)      { acc += alpha * (amp[i] - acc); amp[i] = acc; }
    for (size_t i = amp_len; i-- > 0; )       { acc += alpha * (amp[i] - acc); amp[i] = acc; }

    /* normalization from the maximum */
    double max = 0.0;
    for (size_t i = 0; i < amp_len; i++) max = MAX(max, amp[i]);

    if (max < dsp_data.amplitude_floor)
    {
        memset(amp, 0, amp_len * sizeof(double));
        return;
    }

    /* gate + scaling */
    for (size_t i = 0; i < amp_len; i++)
    {
        double g = (amp[i] / max - SQUELCH_RATIO) / (1.0 - SQUELCH_RATIO);
        amp[i] = (g > 0.0) ? OUTPUT_AMPLITUDE * g : 0.0;
    }
}

/**
 * @brief Map the instantaneous frequency of the input signal (which is in the radio base-band)
 * to the audio-band
 * @param [in/out] freq the instantaneous frequency of the signal to map in the audio band
 * @param [in]     N    the length of the signal
 */
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

/**
 * @brief Stretch a signal on the time axis and with a given factor
 * @param [in]  source_signal      the source signal to stretch
 * @param [in]  source_length      the length of the source signal
 * @param [out] destination_signal the output stretched signal
 * @param [out] destination_length the length of the destination signal
 * @param [in]  stretch_factor     the factor to which the signal will be stretched
 * @return the destination signal (the source signal, stretched)
 */
static void time_stretch(const double *source_signal, const size_t source_length, double **destination_signal,
                         size_t *destination_length, const double stretch_factor)
{
    if (source_signal == NULL || destination_length == NULL)
    {
        fprintf(stderr, "[processing] time_stretch: received NULL pointer.\n");
        return;
    }

    if (source_length < 2 || stretch_factor <= 1.0)
    {
        fprintf(stderr, "[processing] time_stretch: received invalid source_length.\n");
        return;
    }

    const double x_duration  = (double) source_length / dsp_data.downsampled_fs;
    const double y_duration  = x_duration * stretch_factor;
    double n_exact           = round(y_duration * dsp_data.output_fs);
    if (n_exact < 2.0) n_exact = 2.0;
    const size_t n_len = (size_t) n_exact;

    double *x_time = malloc(source_length * sizeof(double));
    double *y_time = malloc(n_len * sizeof(double));
    if (x_time == NULL || y_time == NULL)
    {
        free(x_time);
        free(y_time);
        return;
    }

    /* Making the time axis for the input signal, x */
    for (size_t i = 0; i < source_length; i++)
    {
        x_time[i] = (double) i / dsp_data.downsampled_fs;
    }

    /* Making the time axis for the output signal, y */
    for (size_t i = 0; i < n_len; i++)
    {
        y_time[i] = (double) i / dsp_data.output_fs / stretch_factor;
    }

    double *stretched = linear_interpolation(x_time, source_signal, source_length, y_time, n_len);

    free(x_time);
    free(y_time);

    if (stretched == NULL)
    {
        return;
    }

    *destination_length = n_len;
    *destination_signal = stretched;
}

/**
 * @brief Interpolate the source signal on the time points given in argument
 * @param [in] source_time        the x-vector of the source signal
 * @param [in] source_signal      the y-vector of the source signal
 * @param [in] source_length      the length of the source signal
 * @param [in] destination_time   the x-vector of the destination signal
 * @param [in] destination_length the length of the destination signal
 * @return the destination signal
 */
static double* linear_interpolation(const double* source_time, const double* source_signal, const size_t source_length, const double* destination_time, const size_t destination_length)
{
    if (source_time == NULL || source_signal == NULL || destination_time == NULL)
    {
        fprintf(stderr, "[processing] linear_interpolation: received NULL pointer.\n");
        return NULL;
    }

    if (source_length < 2 || destination_length == 0)
    {
        fprintf(stderr, "[processing] linear_interpolation: received an invalid array length.\n");
        return NULL;
    }

    double* destination_signal = malloc(destination_length * sizeof(double));
    if (destination_signal == NULL)
    {
        fprintf(stderr, "[processing] linear_interpolation: failed memory allocation.\n");
        return NULL;
    }

    size_t yi = 0;

    // Left-side extrapolation: points that are before x_time[0]
    while (yi < destination_length && destination_time[yi] < source_time[0])
    {
        interpolate_value(source_time, source_signal, destination_time, destination_signal, 0, 1, yi);
        yi++;
    }

    if (yi >= destination_length) return destination_signal;

    // Interpolation: points that are in range [xi, xi+1[
    for (size_t xi = 0; xi + 1 < source_length; xi++)
    {
        while (yi < destination_length && destination_time[yi] >= source_time[xi] && destination_time[yi] < source_time[xi+1])
        {
            interpolate_value(source_time, source_signal, destination_time, destination_signal, xi, xi+1, yi);
            yi++;
        }
    }

    // Right-side extrapolation: points that are after x[x_len - 1]
    while (yi < destination_length)
    {
        interpolate_value(source_time, source_signal, destination_time, destination_signal, source_length - 2, source_length - 1, yi);
        yi++;
    }

    return destination_signal;
}

/**
 * @brief Calculate the first-order linear interpolation at the index of the destination signal
 * @param [in]  source_time        the x-vector of the source signal
 * @param [in]  source_signal      the y-vector of the source signal
 * @param [in]  destination_time   the x-vector of the destination signal
 * @param [out] destination_signal the y-vector of the destination signal
 * @param [in]  left_idx           the left point for the interpolation
 * @param [in]  right_idx          the right point for the interpolation
 * @param [in]  destination_idx    the index at which we must right the interpolated value
 */
static void interpolate_value(const double* source_time, const double* source_signal, const double* destination_time, double* destination_signal, const
                              size_t left_idx, const size_t right_idx, const size_t destination_idx)
{
    const double dx    = source_time[right_idx] - source_time[left_idx];
    const double slope = (dx != 0.0) ? (source_signal[right_idx] - source_signal[left_idx]) / dx : 0.0;
    destination_signal[destination_idx] = source_signal[left_idx] + slope * (destination_time[destination_idx] - source_time[left_idx]);
}

/**
 * @brief Generate an audio signal from of the instantaneous frequency and amplitude.
 * @param [in]  freq_t        instantaneous frequency (hz)
 * @param [in]  freq_len      length of the frequency array
 * @param [in]  amp_envelope  instantaneous amplitude (same length as frequency)
 * @param [out] output_signal the audio signal
 */
static void audio_render(const double* freq_t, const size_t freq_len, const double *amp_envelope, double **output_signal)
{
    if (freq_t == NULL || amp_envelope == NULL || freq_len < 2)
    {
        fprintf(stderr, "[processing] audio_render: received NULL pointer or invalid array length.\n");
        return;
    }

    double* out = malloc(freq_len * sizeof(double));
    if (out == NULL) return;

    const double sample_period = 1 / dsp_data.output_fs;
    const double nyquist = 0.5 / sample_period;
    double phase = 0.0;

    for (size_t i = 0; i < freq_len; i++)
    {
        out[i] = clip(amp_envelope[i], -1, 1) * sin(phase);
        // y2[i] = OUTPUT_AMPLITUDE * sin(phase);
        // integration of frequency (trapezoid)
        double fi = freq_t[i];
        double fp = (i > 0) ? freq_t[i - 1] : freq_t[0];
        double favg = 0.5 * (fi + fp);

        if (favg < 0.0) favg = 0.0;
        if (favg > nyquist) favg = nyquist;   // prevent aliasing

        phase += TWOPI * favg * sample_period;
        if (phase >= TWOPI) phase -= TWOPI;
    }

    /* Applying fade-in / fade-out */
    size_t fade_length = (size_t) round(0.1 * dsp_data.output_fs);
    if (2 * fade_length > freq_len) fade_length = freq_len / 2;

    for (size_t i = 0; i < fade_length; i++)
    {
        /* using a raised cosine (rather than a straight ramp) to not generate high harmonics */
        double amp = 0.5 * (1.0 - cos(M_PI * (i + 0.5) / fade_length));
        out[i] *= amp;
        out[freq_len - 1 - i] *= amp;
    }

    *output_signal = out;
}

/**
 * @brief Returns a value clipped between lower and upper bounds
 * @param val   (in) value to clip in between the lower and upper value
 * @param lower (in) lower limit
 * @param upper (in) upper limit
 * @return val clipped between lower and upper limits
 */
static double clip(double val, const double lower, const double upper)
{
    // Clip val between lower and upper bounds
    val = val < lower ? lower : val;
    val = val > upper ? upper : val;
    return val;
}
