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
 * @file output.c
 * @brief Audio output module implementation: ALSA playback and WAV export.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <sndfile.h>
#include <alsa/asoundlib.h>
#include "output.h"

/** Playback channel count. Most USB interfaces reject mono streams. */
#define OUTPUT_CHANNELS   2u
/** Base name of the auto-incremented WAV files. */
#define FILEPATH          "caudiora"
/** Size of the WAV file name buffer. */
#define MAX_FILENAME_SIZE 30
/** Buffering margin requested from ALSA, in microseconds. */
#define OUTPUT_LATENCY_US 100000u
/** Fade length applied to both ends of a played block, in milliseconds. */
#define OUTPUT_FADE_MS    5u
/** Frame capacity of the double to int16 conversion buffer. */
#define STAGING_FRAMES    2048u

typedef struct
{
    uint16_t recording_number;
    size_t   filepath_len;
    char     full_filepath[MAX_FILENAME_SIZE];

    unsigned int audio_fs;
    snd_pcm_t   *pcm;
    int16_t      staging[STAGING_FRAMES * OUTPUT_CHANNELS];

} output_data_t;

static output_data_t output_data;

/***********************************************************************************************************************
 * Private functions declaration
 **********************************************************************************************************************/

/**
 * @brief Convert one normalized sample to signed 16-bit PCM.
 *
 * @param x    Sample normalized to [-1, 1].
 * @param gain Gain applied before conversion.
 * @return The clipped, converted sample.
 */
static int16_t to_pcm16(double x, double gain);

/**
 * @brief Compute the fade gain of a sample within a block.
 *
 * @param i    Index of the sample in the block.
 * @param n    Block length.
 * @param fade Fade length, in samples.
 * @return A gain in [0, 1]: a ramp over both ends of the block, 1.0 in between.
 */
static double fade_gain(size_t i, size_t n, size_t fade);

/**
 * @brief Write an interleaved buffer to the playback device.
 *
 * Retries until every frame is written, recovering from underruns.
 *
 * @param pcm    Opened playback handle.
 * @param buf    Interleaved ::OUTPUT_CHANNELS-channel buffer.
 * @param frames Number of frames to write.
 * @return 0 on success, -1 on unrecoverable failure.
 */
static int write_frames(snd_pcm_t *pcm, const int16_t *buf, snd_pcm_uframes_t frames);

/**
 * @brief Move the auto-incremented WAV file name to the next recording.
 */
static void update_filepath(void);

/***********************************************************************************************************************
 * Public functions implementation
 **********************************************************************************************************************/

int output_open(const char *device, const unsigned int fs)
{
    if (fs == 0)
        return -1;

    /* snd_pcm_open() allocates the handle itself and returns it. */
    snd_pcm_t *pcm = NULL;

    const char *name = (device != NULL && device[0] != '\0') ? device : "default";

    int err = snd_pcm_open(&pcm, name, SND_PCM_STREAM_PLAYBACK, 0);
    if (err < 0) {
        fprintf(stderr, "output: cannot open '%s' (%s)\n",
                name, snd_strerror(err));
        return -1;
    }

    /* The soft_resample argument is set so ALSA resamples when the card does
     * not support fs natively. */
    err = snd_pcm_set_params(pcm,
                             SND_PCM_FORMAT_S16_LE,
                             SND_PCM_ACCESS_RW_INTERLEAVED,
                             OUTPUT_CHANNELS,
                             fs,
                             1,
                             OUTPUT_LATENCY_US);
    if (err < 0) {
        fprintf(stderr,
                "output: cannot configure '%s' at %u Hz (%s)\n"
                "        try a plughw: device name instead of hw:\n",
                name, fs, snd_strerror(err));
        snd_pcm_close(pcm);
        return -1;
    }

    output_data.audio_fs = fs;
    output_data.pcm      = pcm;

    return 0;
}

int output_play(const double *samples, size_t n_frames)
{
    if (output_data.pcm == NULL || samples == NULL || n_frames == 0)
    {
        return -1;
    }

    /* snd_pcm_drain() leaves the stream in the SETUP state, so it has to be
     * prepared again before the next block. */
    snd_pcm_state_t state = snd_pcm_state(output_data.pcm);
    if (state != SND_PCM_STATE_PREPARED && state != SND_PCM_STATE_RUNNING) {
        int err = snd_pcm_prepare(output_data.pcm);
        if (err < 0) {
            fprintf(stderr, "output: cannot prepare stream (%s)\n",
                    snd_strerror(err));
            return -1;
        }
    }

    const size_t fade = (size_t)output_data.audio_fs * OUTPUT_FADE_MS / 1000u;

    for (size_t i = 0; i < n_frames; ) {
        size_t chunk = n_frames - i;
        if (chunk > STAGING_FRAMES)
            chunk = STAGING_FRAMES;

        for (size_t j = 0; j < chunk; j++) {
            int16_t v = to_pcm16(samples[i + j], fade_gain(i + j, n_frames, fade));

            for (unsigned c = 0; c < OUTPUT_CHANNELS; c++)
                output_data.staging[j * OUTPUT_CHANNELS + c] = v;
        }

        if (write_frames(output_data.pcm, output_data.staging, (snd_pcm_uframes_t)chunk) < 0)
            return -1;

        i += chunk;
    }

    /* Blocks until the last sample has left the sound card. */
    int err = snd_pcm_drain(output_data.pcm);
    if (err < 0) {
        fprintf(stderr, "output: cannot drain stream (%s)\n", snd_strerror(err));
        return -1;
    }

    return 0;
}

void output_close(void)
{
    if (output_data.pcm == NULL)
        return;

    /* snd_pcm_close() destroys the handle allocated by snd_pcm_open(): no
     * free() here, that would be a double free. */
    snd_pcm_close(output_data.pcm);
    output_data.pcm = NULL;
}

void output_list_devices(void)
{
    printf("Alsa peripherals :\n");
    printf("  %-24s  %s\n", "default", "system default output");

    int card = -1;

    while (snd_card_next(&card) == 0 && card >= 0) {
        char ctl_name[32];
        snprintf(ctl_name, sizeof ctl_name, "hw:%d", card);

        snd_ctl_t *ctl;
        if (snd_ctl_open(&ctl, ctl_name, 0) < 0)
            continue;

        snd_ctl_card_info_t *card_info;
        snd_ctl_card_info_alloca(&card_info);

        if (snd_ctl_card_info(ctl, card_info) < 0) {
            snd_ctl_close(ctl);
            continue;
        }

        int dev = -1;

        while (snd_ctl_pcm_next_device(ctl, &dev) == 0 && dev >= 0) {
            snd_pcm_info_t *pcm_info;
            snd_pcm_info_alloca(&pcm_info);

            snd_pcm_info_set_device(pcm_info, (unsigned)dev);
            snd_pcm_info_set_subdevice(pcm_info, 0);
            snd_pcm_info_set_stream(pcm_info, SND_PCM_STREAM_PLAYBACK);

            /* Fails when the device has no playback stream: skip it. */
            if (snd_ctl_pcm_info(ctl, pcm_info) < 0)
                continue;

            char name[32];
            snprintf(name, sizeof name, "plughw:%s,%d",
                     snd_ctl_card_info_get_id(card_info), dev);

            printf("  %-24s  %s - %s\n", name,
                   snd_ctl_card_info_get_name(card_info),
                   snd_pcm_info_get_name(pcm_info));
        }

        snd_ctl_close(ctl);
    }
}

void init_wav_export(void)
{
    output_data.recording_number = 0;
    snprintf(output_data.full_filepath, MAX_FILENAME_SIZE, "%s", FILEPATH);
    output_data.filepath_len = strlen(output_data.full_filepath);

    snprintf(output_data.full_filepath + output_data.filepath_len,
             MAX_FILENAME_SIZE - output_data.filepath_len,
             "_%u.wav", output_data.recording_number);
}

int write_wav(char *path, const double *samples, size_t frame_count)
{
    char* filepath;
    if (path == NULL)
    {
        update_filepath();
        filepath = output_data.full_filepath;
    }
    else
    {
        filepath = path;
    }

    SF_INFO info = {0};
    info.samplerate = (output_data.audio_fs != 0)
                          ? (int)output_data.audio_fs
                          : (int)AUDIO_SAMPLE_RATE;
    info.channels   = 1;
    info.format     = SF_FORMAT_WAV | SF_FORMAT_PCM_16;

    SNDFILE *f = sf_open(filepath, SFM_WRITE, &info);
    if (!f) {
        fprintf(stderr, "sf_open(%s): %s\n", filepath, sf_strerror(NULL));
        return -1;
    }

    sf_count_t written = sf_writef_double(f, samples, (sf_count_t)frame_count);
    sf_close(f);

    return (written == (sf_count_t)frame_count) ? 0 : -1;
}

/***********************************************************************************************************************
 * Private functions implementations
 **********************************************************************************************************************/

static void update_filepath(void)
{
    output_data.recording_number++;

    snprintf(output_data.full_filepath + output_data.filepath_len,
             MAX_FILENAME_SIZE - output_data.filepath_len,
             "_%u.wav", output_data.recording_number);
}

static int16_t to_pcm16(double x, double gain)
{
    double v = x * gain;

    /* Explicit clipping: a value slightly above 1.0 would silently wrap to a
     * large negative sample. */
    if (v > 1.0)
        v = 1.0;
    else if (v < -1.0)
        v = -1.0;

    return (int16_t)lrint(v * 32767.0);
}

static double fade_gain(size_t i, size_t n, size_t fade)
{
    if (fade == 0 || 2 * fade >= n)
        return 1.0;

    if (i < fade)
        return (double)i / (double)fade;

    if (i >= n - fade)
        return (double)(n - 1 - i) / (double)fade;

    return 1.0;
}

static int write_frames(snd_pcm_t *pcm, const int16_t *buf, snd_pcm_uframes_t frames)
{
    while (frames > 0) {
        snd_pcm_sframes_t written = snd_pcm_writei(pcm, buf, frames);

        if (written < 0) {
            /* Handles underruns and resume after suspend; 1 means silent. */
            int err = snd_pcm_recover(pcm, (int)written, 1);
            if (err < 0) {
                fprintf(stderr, "output: write failed (%s)\n",
                        snd_strerror(err));
                return -1;
            }
            continue;
        }

        buf    += (size_t)written * OUTPUT_CHANNELS;
        frames -= (snd_pcm_uframes_t)written;
    }

    return 0;
}
