#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include "output.h"
#include "radio_interface.h"
#include "acquisition.h"
#include "processing.h"

// files path in the rapsberry pi: /tmp/tmp.4rshfrT8DF/caudiora/cmake-build-release-raspberrypi

/*
 * To run the project:
 *      - open powershell, connect to rpi by ssh: $ ssh tba@192.168.1.63
 *      - in CLion, open CMake tab (bottom left button) and click on 'Reload CMake Project'
 *      - Run the project (it will send the files to the RPi with ssh, then Rpi will build the project and run the program)
 */

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
    pthread_create(&radio_thread, NULL, radio_stream_start, NULL);

    pthread_t processing_thread;
    pthread_create(&processing_thread, NULL, processing_start, NULL);

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



