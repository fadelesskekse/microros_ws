#ifndef CONTROLLER_CONTROLLERS_H
#define CONTROLLER_CONTROLLERS_H

#include <stddef.h>
#include <stdbool.h>
#include <string.h>

typedef enum
{
    CONTROLLER_STATUS_OK = 0,
    CONTROLLER_STATUS_INVALID_ARGUMENT,
    CONTROLLER_STATUS_CAPACITY_EXCEEDED,
    CONTROLLER_STATUS_INITIALIZATION_FAILED,
    CONTROLLER_STATUS_NOT_FOUND,
    CONTROLLER_STATUS_INVALID_CONTROLLER_TYPE,
    CONTROLLER_STATUS_NOT_INITIALIZED,
    CONTROLLER_STATUS_CALCULATION_FAILED,
    CONTROLLER_STATUS_NO_ACTIVE_CONTROLLER
} controller_status_t;

typedef enum
{
    CONTROLLER_TYPE_NONE = 0,
    CONTROLLER_TYPE_LQR,
    CONTROLLER_TYPE_PID,
    CONTROLLER_TYPE_TEST
} controller_type_t;

#ifdef CONTROLLER_LQR_ENABLED
controller_status_t controller_register_lqr(
    const char *name,
    const double *gain_matrix,
    size_t gain_rows,
    size_t gain_columns
);
#endif

controller_status_t selected_controllers_register(void);

controller_status_t controllers_initialize(void);

controller_status_t controller_activate(
    controller_type_t controller_type,
    const char *controller_name
);

controller_status_t controller_calculate_active(
    const double *state,
    size_t state_length,
    double *output,
    size_t output_length
);

size_t controller_get_active_output_length(void);

#endif