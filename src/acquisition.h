//
// Created by thoma on 30/08/2026.
//

#ifndef CAUDIORA_ACQUISITION_H
#define CAUDIORA_ACQUISITION_H

#include <stddef.h>
#include <stdint.h>

#define RECORDING_BUFFER_SIZE 16777216

int acquisition_init(void);
static void on_iq_samples(const uint8_t *iq, const size_t len, void *user_ctx);

#endif //CAUDIORA_ACQUISITION_H
