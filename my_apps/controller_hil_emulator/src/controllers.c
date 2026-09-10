#include "controller/controllers.h"

static controller_type_t active_controller_type =
    CONTROLLER_TYPE_NONE;

static size_t active_controller_output_length;

#ifdef CONTROLLER_LQR_ENABLED

#include "controller/lqr.h"

#define MAX_LQR_CONTROLLERS 3U

typedef struct
{
    const char *name;
    lqr_controller_t controller;
} named_lqr_controller_t;


// Have these inside an ifdef preprocessor statement as we dont need them
// if we didn't define any lqr controllers. in the future. 

static named_lqr_controller_t *active_lqr_controller;

static named_lqr_controller_t
    lqr_controllers[MAX_LQR_CONTROLLERS];

static size_t lqr_controller_count;


controller_status_t controller_register_lqr(
    const char *name,
    const double *gain_matrix,
    size_t gain_rows,
    size_t gain_columns)
{
    if (name == NULL || gain_matrix == NULL) {
        return CONTROLLER_STATUS_INVALID_ARGUMENT;
    }

    if (lqr_controller_count >= MAX_LQR_CONTROLLERS) {
        return CONTROLLER_STATUS_CAPACITY_EXCEEDED;
    }

    named_lqr_controller_t *entry =
        &lqr_controllers[lqr_controller_count];

    entry->name = name;

    lqr_status_t lqr_status = lqr_controller_init(
        &entry->controller,
        gain_matrix,
        gain_rows,
        gain_columns
    );

    if (lqr_status != LQR_STATUS_OK) {
        return CONTROLLER_STATUS_INITIALIZATION_FAILED;
    }

    if (lqr_controller_count == 0U) {
    active_lqr_controller = entry;

    if (active_controller_type == CONTROLLER_TYPE_NONE) {
    active_controller_type = CONTROLLER_TYPE_LQR;

    active_controller_output_length =
        active_lqr_controller->controller.gain_rows;
    }       //Chatgpt please see this: I will need to extend this functinoality to other controllers init process
    } // Default lqr active as the first entry (0th entry)

    ++lqr_controller_count;

    return CONTROLLER_STATUS_OK;
}


static controller_status_t activate_lqr_controller(
    const char *controller_name)
{
    if (controller_name == NULL) {
        return CONTROLLER_STATUS_INVALID_ARGUMENT;
    }

    for (size_t i = 0; i < lqr_controller_count; ++i) {
        if (strcmp(
                lqr_controllers[i].name,
                controller_name
            ) == 0)
        {
            active_lqr_controller = &lqr_controllers[i];
            active_controller_type = CONTROLLER_TYPE_LQR;

            active_controller_output_length =
                active_lqr_controller->controller.gain_rows;
                // Will also  need to set this for other controllers. 
            return CONTROLLER_STATUS_OK;
        }
    }

    return CONTROLLER_STATUS_NOT_FOUND;
}

#endif

#ifdef CONTROLLER_TEST_ENABLED
#include "controller/test.h"
#endif


static bool controllers_initialized;

controller_status_t controllers_initialize(void)
{
    controllers_initialized = false;
    active_controller_type = CONTROLLER_TYPE_NONE;
    active_controller_output_length = 0U;

    #ifdef CONTROLLER_LQR_ENABLED
        lqr_controller_count = 0U;
        active_lqr_controller = NULL;
    #endif

    #ifdef CONTROLLER_PID_ENABLED
        pid_controller_count = 0U;
        active_pid_controller = NULL;
    #endif

    controller_status_t status =
        selected_controllers_register();

    if (status != CONTROLLER_STATUS_OK) {
        return status;
    }

    controllers_initialized = true;

    return CONTROLLER_STATUS_OK;
}

controller_status_t controller_activate(
    controller_type_t controller_type,
    const char *controller_name)
{
    switch (controller_type) {
#ifdef CONTROLLER_LQR_ENABLED
        case CONTROLLER_TYPE_LQR:
            return activate_lqr_controller(controller_name);
#endif

#ifdef CONTROLLER_PID_ENABLED
        case CONTROLLER_TYPE_PID:
            return activate_pid_controller(controller_name);
#endif

#ifdef CONTROLLER_TEST_ENABLED
        case CONTROLLER_TYPE_TEST:
            return activate_test_controller(controller_name);
#endif

        case CONTROLLER_TYPE_NONE:
        default:
            return CONTROLLER_STATUS_INVALID_CONTROLLER_TYPE;
    }
}

controller_status_t controller_calculate_active(
    const double *state,
    size_t state_length,
    double *output,
    size_t output_length)
{
    if (!controllers_initialized) {
        return CONTROLLER_STATUS_NOT_INITIALIZED;
    }

    switch (active_controller_type) {
#ifdef CONTROLLER_LQR_ENABLED
        case CONTROLLER_TYPE_LQR: {
            if (active_lqr_controller == NULL) {
                return CONTROLLER_STATUS_NOT_INITIALIZED;
            }

            lqr_status_t status =
                lqr_controller_calculate(
                    &active_lqr_controller->controller,
                    state,
                    state_length,
                    output,
                    output_length
                );

            return status == LQR_STATUS_OK
                ? CONTROLLER_STATUS_OK
                : CONTROLLER_STATUS_CALCULATION_FAILED;
        }
#endif

#ifdef CONTROLLER_PID_ENABLED
        case CONTROLLER_TYPE_PID:
            /* Call pid_controller_calculate() here. */
            return CONTROLLER_STATUS_CALCULATION_FAILED;
#endif

        default:
            return CONTROLLER_STATUS_NO_ACTIVE_CONTROLLER;
    }
}

size_t controller_get_active_output_length(void)
{
    return active_controller_output_length;
}