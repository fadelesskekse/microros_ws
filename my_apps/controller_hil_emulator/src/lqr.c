#include "controller/lqr.h"

lqr_status_t lqr_controller_init(
    lqr_controller_t *controller,
    const double *gain_matrix,
    size_t gain_rows,
    size_t gain_columns)
{
    if (controller == NULL) {
        return LQR_STATUS_NULL_POINTER;
    }

    /*
     * Leave the controller in a known, uninitialized state if
     * validation fails.
     */
    controller->gain_matrix = NULL;
    controller->gain_rows = 0U;
    controller->gain_columns = 0U;

    if (gain_matrix == NULL) {
        return LQR_STATUS_NULL_POINTER;
    }

    if (gain_rows == 0U || gain_columns == 0U) {
        return LQR_STATUS_INVALID_DIMENSIONS;
    }

    controller->gain_matrix = gain_matrix;
    controller->gain_rows = gain_rows;
    controller->gain_columns = gain_columns;

    return LQR_STATUS_OK;
}

lqr_status_t lqr_controller_calculate(
    const lqr_controller_t *controller,
    const double *state,
    size_t state_length,
    double *output,
    size_t output_length
)
{
    if (
        controller == NULL ||
        state == NULL ||
        output == NULL
    ) {
        return LQR_STATUS_NULL_POINTER;
    }

    if (controller->gain_matrix == NULL) {
        return LQR_STATUS_NULL_POINTER;
    }

    if (
        state_length != controller->gain_columns ||
        output_length != controller->gain_rows
    ) {
        return LQR_STATUS_SIZE_MISMATCH;
    }

    for (size_t row = 0; row < controller->gain_rows; ++row) {
        double value = 0.0;

        for (
            size_t column = 0;
            column < controller->gain_columns;
            ++column
        ) {
            size_t gain_index =
                row * controller->gain_columns + column;

            value +=
                controller->gain_matrix[gain_index] *
                state[column];
        }

        output[row] = -value;
    }

    return LQR_STATUS_OK;
}