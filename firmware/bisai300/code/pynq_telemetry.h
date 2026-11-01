#ifndef PYNQ_TELEMETRY_H
#define PYNQ_TELEMETRY_H

#include <stdint.h>

/* All distances are transmitted in 0.001 field-grid units, not metres.
 * Physical grid size is applied by the PYNQ receiver after confirmation.
 * Exactly 16 bytes per 100 ms; see PROTOCOL.md for the wire format.
 */
void pynq_telemetry_sample(float dx_grid, float dy_grid,
                         float x_grid, float y_grid, float heading_deg);
void pynq_telemetry_poll(void);
void pynq_raw_encoder(const volatile int16_t *counts);
void pynq_raw_gyro(int16_t z);

#endif
