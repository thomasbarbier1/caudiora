#include <sndfile.h>
#include <stdio.h>
#include <string.h>
#include "output.h"

#define FILEPATH "caudiora"
#define MAX_FILENAME_SIZE 30

typedef struct
{
    uint16_t recording_number;
    size_t   filepath_len;
    char     full_filepath[MAX_FILENAME_SIZE];
} output_t;

static output_t output_data;

void init_output(void)
{
    output_data.recording_number = 0;
    snprintf(output_data.full_filepath, MAX_FILENAME_SIZE, FILEPATH);
    output_data.filepath_len = strlen(FILEPATH);
    snprintf(output_data.full_filepath + output_data.filepath_len,
        MAX_FILENAME_SIZE,
         "_%u.wav", output_data.recording_number);
}

static void update_filepath()
{
    output_data.recording_number++;
    snprintf(output_data.full_filepath + output_data.filepath_len,
        MAX_FILENAME_SIZE,
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
    info.samplerate = AUDIO_SAMPLE_RATE;
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