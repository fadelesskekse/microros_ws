#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microros/rmw_microros.h>
#include <microros_transports.h>

#include <sensor_msgs/msg/joint_state.h>
#include <control_framework_interfaces/msg/control_input.h>
#include <control_framework_interfaces/srv/controller_select.h>

#include <controller/controllers.h>
#include "joint_filter_config.h"
//In Cmake, this directory is specified by the model I provide in its build arguements 
//Later I will need to specify if I am using a simulator or real like so I dont include this as
//Real Implementation will come from sensors directly interfacing with this. 

#include <stdio.h>
#include <unistd.h>
#include <time.h>

#include <zephyr.h>

#define STRING_BUFFER_LEN 50
#define CONTROLLER_NAME_BUFFER_LENGTH 50U
#define SERVICE_MESSAGE_BUFFER_LENGTH 100U

#define CONTROLLER_STATE_LENGTH \
    (2U * JOINT_FILTER_CONTROLLED_JOINT_COUNT)

#define MAX_CONTROL_OUTPUT_LENGTH 4U

#define MAX_JOINT_NAME_LENGTH 50U



#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t control_publisher_;
rcl_subscription_t state_subscriber_;
rcl_service_t controller_select_service_;

sensor_msgs__msg__JointState joint_state_;
control_framework_interfaces__msg__ControlInput control_input_;

control_framework_interfaces__srv__ControllerSelect_Request
    controller_select_request_;

control_framework_interfaces__srv__ControllerSelect_Response
    controller_select_response_;

static char controller_name_buffer[
    CONTROLLER_NAME_BUFFER_LENGTH
];

static char service_message_buffer[
    SERVICE_MESSAGE_BUFFER_LENGTH
];

static double controller_state_[
    CONTROLLER_STATE_LENGTH //the size needs to change. 
];

static double controller_output_[
    MAX_CONTROL_OUTPUT_LENGTH
];

static double control_input_buffer_[
    MAX_CONTROL_OUTPUT_LENGTH
];

static double joint_position_buffer_[
    JOINT_FILTER_TOTAL_JOINT_COUNT
];

static double joint_velocity_buffer_[
    JOINT_FILTER_TOTAL_JOINT_COUNT
];

static rosidl_runtime_c__String joint_name_strings_[
    JOINT_FILTER_TOTAL_JOINT_COUNT
];

static char joint_name_buffers_[
    JOINT_FILTER_TOTAL_JOINT_COUNT
][MAX_JOINT_NAME_LENGTH];

static char joint_frame_id_buffer_[1U];


void controller_select_callback(
    const void *request_message,
    void *response_message)
{
    const control_framework_interfaces__srv__ControllerSelect_Request
        *request =
            (const control_framework_interfaces__srv__ControllerSelect_Request *)
                request_message;

    control_framework_interfaces__srv__ControllerSelect_Response
        *response =
            (control_framework_interfaces__srv__ControllerSelect_Response *)
                response_message;

    if (request == NULL ||
        response == NULL ||
        request->controller_name.data == NULL)
    {
        return;
    }

    controller_type_t requested_type;

    switch (request->controller_type) {
        case control_framework_interfaces__srv__ControllerSelect_Request__CONTROLLER_TYPE_LQR:
            requested_type = CONTROLLER_TYPE_LQR;
            break;

        case control_framework_interfaces__srv__ControllerSelect_Request__CONTROLLER_TYPE_PID:
            requested_type = CONTROLLER_TYPE_PID;
            break;

        case control_framework_interfaces__srv__ControllerSelect_Request__CONTROLLER_TYPE_TEST:
            requested_type = CONTROLLER_TYPE_TEST;
            break;

        case control_framework_interfaces__srv__ControllerSelect_Request__CONTROLLER_TYPE_NONE:
        default:
            response->success = false;

            snprintf(
                response->message.data,
                response->message.capacity,
                "Invalid controller type"
            );

            response->message.size =
                strlen(response->message.data);

            return;
    }

    const controller_status_t status =
        controller_activate(
            requested_type,
            request->controller_name.data
        );

    response->success =
        status == CONTROLLER_STATUS_OK;

    if (response->success) {
        snprintf(
            response->message.data,
            response->message.capacity,
            "Activated controller: %s",
            request->controller_name.data
        );
    } else {
        snprintf(
            response->message.data,
            response->message.capacity,
            "Controller activation failed: %d",
            (int)status
        );
    }

    response->message.size =
        strlen(response->message.data);
}

void control_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	(void) last_call_time;

	if(timer == NULL){
		return;
	}

	const size_t output_length = controller_get_active_output_length();

    if (output_length == 0U ||
        output_length > MAX_CONTROL_OUTPUT_LENGTH ||
        output_length > control_input_.control_input.capacity)
    {
        return;
    }

	controller_status_t status =
    controller_calculate_active(
        controller_state_,
        CONTROLLER_STATE_LENGTH,
        controller_output_,
        output_length
    );

	    if (status != CONTROLLER_STATUS_OK) {
        return;
    }

    for (size_t i = 0; i < output_length; ++i) {
        control_input_.control_input.data[i] =
            controller_output_[i];
    }

    control_input_.control_input.size =
        output_length;

    RCSOFTCHECK(
        rcl_publish(
            &control_publisher_,
            &control_input_,
            NULL
        )
    );

}

void state_subscription_callback(const void *message)
{
    if (message == NULL) {
        return;
    }

    const sensor_msgs__msg__JointState *joint_state =
        (const sensor_msgs__msg__JointState *)message;

    if (joint_state->position.data == NULL ||
        joint_state->velocity.data == NULL)
    {
        return;
    }

    for (size_t i = 0;
         i < JOINT_FILTER_CONTROLLED_JOINT_COUNT;
         ++i)
    {
        const size_t joint_index =
            JOINT_FILTER_CONTROLLED_JOINT_INDICES[i];

        if (joint_index >= joint_state->position.size ||
            joint_index >= joint_state->velocity.size)
        {
            return;
        }

        controller_state_[2U * i] =
            joint_state->position.data[joint_index];

        controller_state_[2U * i + 1U] =
            joint_state->velocity.data[joint_index];
    }
}


void main(void)
{
	controller_status_t controller_status =
	controllers_initialize();

	if (controller_status != CONTROLLER_STATUS_OK) {
		printf(
			"Controller initialization failed: %d\n",
			(int)controller_status
		);

		return;
	}
		// Set custom transports
	rmw_uros_set_custom_transport(
		MICRO_ROS_FRAMING_REQUIRED,
		(void *) &default_params,
		zephyr_transport_open,
		zephyr_transport_close,
		zephyr_transport_write,
		zephyr_transport_read
	);

	// Init micro-ROS
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "controller_hil_emulator_node", "", &support));

	RCCHECK(
		rclc_publisher_init_default(
			&control_publisher_,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(
				control_framework_interfaces,
				msg,
				ControlInput
			),
			"control_input"
		)
	);

	RCCHECK(
		rclc_subscription_init_default(
			&state_subscriber_,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(
				sensor_msgs,
				msg,
				JointState
			),
			"joint_states"
		)
	);
   // Create a 1ms control timer
	rcl_timer_t timer;
	RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1), control_timer_callback));

	RCCHECK(
    rclc_service_init_default(
        &controller_select_service_,
        &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(
            control_framework_interfaces,
            srv,
            ControllerSelect
        ),
        "/controller_select"
    )
	);

	controller_name_buffer[0] = '\0';

	controller_select_request_.controller_name.data =
		controller_name_buffer;

	controller_select_request_.controller_name.size = 0U;

	controller_select_request_.controller_name.capacity =
		CONTROLLER_NAME_BUFFER_LENGTH;


	service_message_buffer[0] = '\0';

	controller_select_response_.message.data =
		service_message_buffer;

	controller_select_response_.message.size = 0U;

	controller_select_response_.message.capacity =
		SERVICE_MESSAGE_BUFFER_LENGTH;

	control_input_.control_input.data =
    control_input_buffer_;

	control_input_.control_input.size = 0U;

	control_input_.control_input.capacity =
		MAX_CONTROL_OUTPUT_LENGTH;

	joint_state_.position.data =
    joint_position_buffer_;

	joint_state_.position.size = 0U;

	joint_state_.position.capacity =
    JOINT_FILTER_TOTAL_JOINT_COUNT;

	joint_state_.velocity.data =
    joint_velocity_buffer_;

	joint_state_.velocity.size = 0U;

	joint_state_.velocity.capacity =
    JOINT_FILTER_TOTAL_JOINT_COUNT;

	joint_state_.name.data =
    joint_name_strings_;
	joint_state_.name.size = 0U;
	joint_state_.name.capacity =
    JOINT_FILTER_TOTAL_JOINT_COUNT;

	for (size_t i = 0;
		i < JOINT_FILTER_TOTAL_JOINT_COUNT;
		++i)
	{
		joint_name_buffers_[i][0] = '\0';

		joint_name_strings_[i].data =
			joint_name_buffers_[i];

		joint_name_strings_[i].size = 0U;

		joint_name_strings_[i].capacity =
			MAX_JOINT_NAME_LENGTH;
	}

	joint_frame_id_buffer_[0] = '\0';

	joint_state_.header.frame_id.data =
		joint_frame_id_buffer_;
	joint_state_.header.frame_id.size = 0U;
	joint_state_.header.frame_id.capacity = 1U;

	joint_state_.effort.data = NULL;
	joint_state_.effort.size = 0U;
	joint_state_.effort.capacity = 0U;

	rclc_executor_t executor;

	RCCHECK(
		rclc_executor_init(
			&executor,
			&support.context,
			3,
			&allocator
		)
	);

	RCCHECK(
		rclc_executor_add_service(
			&executor,
			&controller_select_service_,
			&controller_select_request_,
			&controller_select_response_,
			controller_select_callback
		)
	);

	RCCHECK(
		rclc_executor_add_subscription(
			&executor,
			&state_subscriber_,
			&joint_state_,
			state_subscription_callback,
			ON_NEW_DATA
		)
	);

	RCCHECK(
		rclc_executor_add_timer(
			&executor,
			&timer
		)
	);

	while (1) {
		RCSOFTCHECK(
			rclc_executor_spin_some(
				&executor,
                RCL_MS_TO_NS(1)
			)
		);
	}
}


// void main(void)
// {
// 	// Set custom transports
// 	rmw_uros_set_custom_transport(
// 		MICRO_ROS_FRAMING_REQUIRED,
// 		(void *) &default_params,
// 		zephyr_transport_open,
// 		zephyr_transport_close,
// 		zephyr_transport_write,
// 		zephyr_transport_read
// 	);

// 	// Init micro-ROS
// 	rcl_allocator_t allocator = rcl_get_default_allocator();
// 	rclc_support_t support;

// 	// create init_options
// 	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

// 	// create node
// 	rcl_node_t node;
// 	RCCHECK(rclc_node_init_default(&node, "pingpong_node", "", &support));

// 	// Create a reliable ping publisher
// 	RCCHECK(rclc_publisher_init_default(&ping_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/ping"));

// 	// Create a best effort pong publisher
// 	RCCHECK(rclc_publisher_init_best_effort(&pong_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/pong"));

// 	// Create a best effort ping subscriber
// 	RCCHECK(rclc_subscription_init_best_effort(&ping_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/ping"));

// 	// Create a best effort  pong subscriber
// 	RCCHECK(rclc_subscription_init_best_effort(&pong_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/pong"));


// 	// Create a 3 seconds ping timer timer,
// 	rcl_timer_t timer;
// 	RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(2000), ping_timer_callback));


// 	// Create executor
// 	rclc_executor_t executor;
// 	RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
// 	RCCHECK(rclc_executor_add_timer(&executor, &timer));
// 	RCCHECK(rclc_executor_add_subscription(&executor, &ping_subscriber, &incoming_ping, &ping_subscription_callback, ON_NEW_DATA));
// 	RCCHECK(rclc_executor_add_subscription(&executor, &pong_subscriber, &incoming_pong, &pong_subscription_callback, ON_NEW_DATA));

// 	// Create and allocate the pingpong messages

// 	char outcoming_ping_buffer[STRING_BUFFER_LEN];
// 	outcoming_ping.frame_id.data = outcoming_ping_buffer;
// 	outcoming_ping.frame_id.capacity = STRING_BUFFER_LEN;

// 	char incoming_ping_buffer[STRING_BUFFER_LEN];
// 	incoming_ping.frame_id.data = incoming_ping_buffer;
// 	incoming_ping.frame_id.capacity = STRING_BUFFER_LEN;

// 	char incoming_pong_buffer[STRING_BUFFER_LEN];
// 	incoming_pong.frame_id.data = incoming_pong_buffer;
// 	incoming_pong.frame_id.capacity = STRING_BUFFER_LEN;

// 	device_id = rand();

// 	while(1){
// 		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
// 		usleep(100000);
// 	}	
	
// 	RCCHECK(rcl_publisher_fini(&ping_publisher, &node));
// 	RCCHECK(rcl_publisher_fini(&pong_publisher, &node));
// 	RCCHECK(rcl_subscription_fini(&ping_subscriber, &node));
// 	RCCHECK(rcl_subscription_fini(&pong_subscriber, &node));
// 	RCCHECK(rcl_node_fini(&node));
// }
