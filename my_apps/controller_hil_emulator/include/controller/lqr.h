#ifndef CONTROLLER_LQR_H
#define CONTROLLER_LQR_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    LQR_STATUS_OK = 0,
    LQR_STATUS_NULL_POINTER,
    LQR_STATUS_INVALID_DIMENSIONS,
    LQR_STATUS_SIZE_MISMATCH
} lqr_status_t;

typedef struct
{
    const double *gain_matrix;
    size_t gain_rows;
    size_t gain_columns;
} lqr_controller_t;

/*
 * Initialize one LQR controller instance.
 *
 * gain_matrix must contain:
 *
 *     gain_rows * gain_columns
 *
 * values in row-major order.
 */
lqr_status_t lqr_controller_init(
    lqr_controller_t *controller,
    const double *gain_matrix,
    size_t gain_rows,
    size_t gain_columns
);

/*
 * Calculate:
 *
 *     output = -(K * state)
 *
 * state_length must equal gain_columns.
 * output_length must equal gain_rows.
 */
lqr_status_t lqr_controller_calculate(
    const lqr_controller_t *controller,
    const double *state,
    size_t state_length,
    double *output,
    size_t output_length
);

#ifdef __cplusplus
}
#endif

#endif