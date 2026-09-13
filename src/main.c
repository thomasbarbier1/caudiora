#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>
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

int main(void)
{
    if (radio_config() != 0)
    {
        printf("Failed to configure the radio. End of program.\n");
        return 1;
    }

    if (radio_init() != 0)
    {
        printf("Failed to initialize radio parameters. End of program.\n");
        return 1;
    }

    if (acquisition_init() != 0)
    {
        printf("Failed to initialize acquisition. End of program.\n");
        return 1;
    }

    if (processing_init() != 0)
    {
        printf("Failed to initialize processing. End of program.\n");
        return 1;
    }

    pthread_t radio_thread;
    pthread_create(&radio_thread, NULL, radio_stream_start, NULL);

    pthread_t processing_thread;
    pthread_create(&processing_thread, NULL, processing_start, NULL);

    sleep(8);
    uint32_t overflow_nb = radio_stream_stop();
    pthread_join(radio_thread, NULL);
    printf("Number of buffer overflow: %u\n", overflow_nb);
    processing_stop();
    pthread_join(processing_thread, NULL);
    printf("End of program.\n");

    return 0;
}



