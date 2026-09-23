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
 * @file main.c
 * @brief Initializes the various modules, starts the threads, and then triggers a timer.
 * When the timer expires, the threads are stopped and the program terminates.
 */

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include "output.h"
#include "radio_interface.h"
#include "acquisition.h"
#include "processing.h"

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        printf("Usage: ./caudiora <duration>\n");
        return EXIT_FAILURE;
    }

    if (radio_config() != 0)
    {
        printf("Failed to configure the radio. End of program.\n");
        return EXIT_FAILURE;
    }

    if (radio_init() != 0)
    {
        printf("Failed to initialize radio parameters. End of program.\n");
        return EXIT_FAILURE;
    }

    if (acquisition_init() != 0)
    {
        printf("Failed to initialize acquisition. End of program.\n");
        return EXIT_FAILURE;
    }

    if (processing_init() != 0)
    {
        printf("Failed to initialize processing. End of program.\n");
        return EXIT_FAILURE;
    }

    if (output_open("plughw:Headphones,0", (unsigned int) 44100) < 0)
    {
        printf("Unable to open audio device\n");
        return EXIT_FAILURE;
    }

    pthread_t radio_thread;
    pthread_create(&radio_thread, NULL, radio_stream_thread, NULL);

    pthread_t processing_thread;
    pthread_create(&processing_thread, NULL, processing_thread, NULL);

    const int duration = atoi(argv[1]);
    sleep(duration);

    uint32_t overflow_nb = radio_stream_stop();
    pthread_join(radio_thread, NULL);
    // printf("Number of buffer overflow: %u\n", overflow_nb);
    processing_stop();
    pthread_join(processing_thread, NULL);
    output_close();

    printf("End of program.\n");

    return 0;
}



