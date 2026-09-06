#ifndef CAUDIORA_OUTPUT_H
#define CAUDIORA_OUTPUT_H

#define AUDIO_SAMPLE_RATE (uint32_t)44100

int write_wav(char *path, const double *samples, size_t frame_count);
void init_output(void);

#endif //CAUDIORA_OUTPUT_H
