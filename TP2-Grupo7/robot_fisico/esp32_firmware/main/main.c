#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "motor_driver.h"
#include "ultrasonic.h"
#include "encoder.h"

#include <std_srvs/srv/set_bool.h>
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/twist.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define MICRO_ROS_APP_STACK 16000
#define MICRO_ROS_APP_TASK_PRIO 5

#define PIN_TRIG        GPIO_NUM_4
#define PIN_ECHO        GPIO_NUM_19
#define MAX_DISTANCE_CM 400
#define AVG_SAMPLES     5
#define ECHO_TIMEOUT_MS 30

#define WHEEL_SEPARATION  0.1f
#define MAX_LINEAR_VEL    1.0f
#define MAX_ANGULAR_VEL   1.0f
#define MAX_WHEEL_SPEED   (MAX_LINEAR_VEL + MAX_ANGULAR_VEL * (WHEEL_SEPARATION / 2.0f))

#define DOMAIN_ID 0

static const char *TAG = "micro_ros";

rcl_service_t servidor_motor; // Servicio para habilitar/deshabilitar motores

rcl_publisher_t publisher_ults; // Publicador para distancia del ultrasonido
rcl_publisher_t publisher_rpm_right; // Publicador para RPM del motor derecho

rcl_subscription_t subscriber; // Suscriptor para cmd_vel (Twist)
 
std_msgs__msg__Float32 pub_msg; // Mensaje para publicar distancia del ultrasonido
std_msgs__msg__Float32 cmd_right; // Mensaje para publicar RPM del motor derecho
//std_msgs__msg__Float32 cmd_left;  // Mensaje para publicar RPM del motor izquierdo (no usado por falta de encoder)

geometry_msgs__msg__Twist cmd_vel_msg; // Mensaje para recibir cmd_vel (Twist)

motor_t motor_right = { .channel = LEDC_CHANNEL_0, .pin_pwm = GPIO_NUM_18, .pin_dir = GPIO_NUM_5  }; // Configuración del motor derecho
motor_t motor_left  = { .channel = LEDC_CHANNEL_1, .pin_pwm = GPIO_NUM_16, .pin_dir = GPIO_NUM_17 }; // Configuración del motor izquierdo

bool motor_right_enabled = true; // Indicador para el estado del motor derecho
bool motor_left_enabled  = true; // Indicador para el estado del motor izquierdo

std_srvs__srv__SetBool_Request  srv_request; // Mensaje para recibir la solicitud del servicio
std_srvs__srv__SetBool_Response srv_response; // Mensaje para enviar la respuesta del servicio


void service_callback(const void *request_msg, void *response_msg)
{
    const std_srvs__srv__SetBool_Request *req =
        (const std_srvs__srv__SetBool_Request *)request_msg;
    std_srvs__srv__SetBool_Response *res =
        (std_srvs__srv__SetBool_Response *)response_msg;

    motor_right_enabled = req->data;                
    motor_left_enabled  = req->data;        // Deshabilita ambos motores si req->data es false, habilita ambos si es true

    if (!req->data) {
        motor_set(&motor_right, 0, MOTOR_FWD);
        motor_set(&motor_left,  0, MOTOR_FWD);  // Detiene ambos motores inmediatamente al deshabilitarlos
    }

    res->success = true;
    ESP_LOGI(TAG, "Motores %s", req->data ? "habilitados" : "deshabilitados");
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time, unsigned int timer_index)
{
    (void) last_call_time;
    (void) timer_index;
    if (timer == NULL)
        return;

    pub_msg.data = (float) ultrasonic_read_filtered_cm(AVG_SAMPLES, PIN_TRIG, PIN_ECHO, ECHO_TIMEOUT_MS, MAX_DISTANCE_CM);
    RCSOFTCHECK(rcl_publish(&publisher_ults, &pub_msg, NULL));

    cmd_right.data = encoder_get_rpm_right();
    //cmd_left.data  = encoder_get_rpm_left(); // no tengo el encoder del motor izquierdo, así que solo publico el derecho por ahora

    RCSOFTCHECK(rcl_publish(&publisher_rpm_right, &cmd_right, NULL));
    //RCSOFTCHECK(rcl_publish(&publisher_rpm_left, &cmd_left, NULL));
}

void subscription_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    float linear_x = msg->linear.x;
    float angular_z = msg->angular.z;

    float v_right = linear_x + (angular_z * WHEEL_SEPARATION / 2.0f);
    float v_left  = linear_x - (angular_z * WHEEL_SEPARATION / 2.0f);

    float max_vel = fmaxf(fabsf(v_right), fabsf(v_left));
    if (max_vel > MAX_WHEEL_SPEED) {
        v_right = (v_right / max_vel) * MAX_WHEEL_SPEED;
        v_left  = (v_left  / max_vel) * MAX_WHEEL_SPEED;
    }

    uint32_t duty_r = (uint32_t)(fabsf(v_right) / MAX_WHEEL_SPEED * MAX_DUTY);
    uint32_t duty_l = (uint32_t)(fabsf(v_left)  / MAX_WHEEL_SPEED * MAX_DUTY);

    motor_dir_t dirr = (v_right >= 0) ? MOTOR_FWD : MOTOR_BWD;
    motor_dir_t dirl = (v_left  >= 0) ? MOTOR_FWD : MOTOR_BWD;

    if (motor_right_enabled)
        motor_set(&motor_right, duty_r, dirr);

    if (motor_left_enabled)
        motor_set(&motor_left, duty_l, dirl);
}

void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t  support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP,
                                             CONFIG_MICRO_ROS_AGENT_PORT,
                                             rmw_options));
#endif

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    rcl_node_t node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "node_name","", &support));
    ESP_LOGI(TAG, "Nodo creado correctamente");

    RCCHECK(rclc_publisher_init_default(
        &publisher_ults,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "ultrasonic_publisher"));

    RCCHECK(rclc_publisher_init_default(
        &publisher_rpm_right,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "rpm_right_publisher"));

    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    rcl_timer_t timer = rcl_get_zero_initialized_timer();
    RCCHECK(rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback,
        true));

    RCCHECK(rclc_service_init_default(
        &servidor_motor,
        &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
        "motors_enable"));

    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator)); //
    RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(1000)));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &cmd_vel_msg,
                                           &subscription_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_service(&executor, &servidor_motor, &srv_request, &srv_response, &service_callback));

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(10000);
    }

    RCCHECK(rcl_service_fini(&servidor_motor, &node));
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_publisher_fini(&publisher_ults, &node));
    RCCHECK(rcl_publisher_fini(&publisher_rpm_right, &node));
    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}

void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    if (ultrasonic_init(PIN_TRIG, PIN_ECHO) != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo inicializar el ultrasonido");
        return;
    }

    motors_init(&motor_right, &motor_left);
    encoders_init();

    xTaskCreate(micro_ros_task, "micro_ros_task",
                MICRO_ROS_APP_STACK, NULL, MICRO_ROS_APP_TASK_PRIO, NULL);
}