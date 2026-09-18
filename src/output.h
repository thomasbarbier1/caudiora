#ifndef CAUDIORA_OUTPUT_H
#define CAUDIORA_OUTPUT_H

#include <stddef.h>
#include <stdint.h>

/**
 * @file output.h
 * @brief Audio output module: ALSA playback and WAV export.
 *
 * Typical usage:
 * @code
 * output_list_devices();
 * if (output_open("plughw:MiniFuse,0", audio_fs) < 0)
 *     return EXIT_FAILURE;
 *
 * init_wav_export();
 * ...
 * write_wav(NULL, samples, n);     // debug capture
 * output_play(samples, n);         // blocks until playback is over
 * ...
 * output_close();
 * @endcode
 */

/** Default audio sample rate, used for WAV export until a playback device is opened. */
#define AUDIO_SAMPLE_RATE (uint32_t)44100

/**
 * @brief Write a mono sample block to a WAV file.
 *
 * The sample rate stored in the file follows the currently opened playback
 * device, so the exported file and the audible output always run at the same
 * speed. Falls back to ::AUDIO_SAMPLE_RATE when no device is open.
 *
 * @param path        Destination file, or NULL to use the auto-incremented
 *                    name set up by ::init_wav_export.
 * @param samples     Mono samples normalized to [-1, 1].
 * @param frame_count Number of samples to write.
 * @return 0 on success, -1 on failure.
 */
int write_wav(char *path, const double *samples, size_t frame_count);

/**
 * @brief Initialize the auto-incremented WAV file naming.
 *
 * Must be called before any ::write_wav call made with a NULL path.
 */
void init_wav_export(void);

/**
 * @brief Print the available ALSA playback devices to stdout.
 *
 * Each line shows the device name to hand over to ::output_open. Devices are
 * enumerated at call time, so a USB interface plugged in after startup only
 * shows up on the next run.
 */
void output_list_devices(void);

/**
 * @brief Open an ALSA playback device.
 *
 * The device is opened in stereo: most USB interfaces reject mono streams, so
 * ::output_play duplicates the mono input onto both channels. Prefer a
 * @c plughw: name over @c hw: so that ALSA transparently converts the sample
 * rate and format the card does not support natively.
 *
 * @param device ALSA device name ("default", "plughw:MiniFuse,0", ...).
 *               NULL or an empty string falls back to "default".
 * @param fs     Sample rate of the blocks to be played, in Hz.
 * @return 0 on success, -1 on failure (details on stderr).
 */
int output_open(const char *device, unsigned int fs);

/**
 * @brief Play a block of mono samples.
 *
 * Blocking: returns only once the last sample has actually left the sound
 * card, which paces the calling thread. A short fade is applied to both ends
 * of the block to suppress the click caused by a non-zero boundary value.
 *
 * @param samples  Mono samples normalized to [-1, 1]; values outside that
 *                 range are clipped.
 * @param n_frames Number of samples to play.
 * @return 0 on success, -1 on failure.
 */
int output_play(const double *samples, size_t n_frames);

/**
 * @brief Close the playback device and release its resources.
 *
 * Safe to call when no device is open.
 */
void output_close(void);

#endif //CAUDIORA_OUTPUT_H
