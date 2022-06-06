#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

// Publishers
rcl_publisher_t pub1;
std_msgs__msg__Int32 msg_pub1; //
rcl_publisher_t pub2;
std_msgs__msg__Int32 msg_pub2; //

// Subscribers
rcl_subscription_t sub1;
std_msgs__msg__Int32 msg_sub1;
rcl_subscription_t sub2;
std_msgs__msg__Int32 msg_sub2;

//Auxiliares
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

////////////////////////////////////////////////////////////////////////
void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&pub1, &msg_pub1, NULL));
    msg_pub1.data++;
    RCSOFTCHECK(rcl_publish(&pub2, &msg_pub2, NULL));
    msg_pub2.data += 2;
  }
}

void subscription_callback_1(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);  
}

void subscription_callback_2(const void * msgin)
{
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  digitalWrite(LED_PIN, (msg->data == -1) ? !(digitalRead(LED_PIN)) : (digitalRead(LED_PIN)));  
}

/**********************************
  SETUP
**********************************/
void setup() {

  delay(5000);
  
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "Odometry", "", &support));

  // create publishers
  RCCHECK(rclc_publisher_init_default(
            &pub1,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "wheel/odometry")); //(parent_folder, sub_folder, msg_type), topic_name

  RCCHECK(rclc_publisher_init_default(
            &pub2,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "imu/data")); //(parent_folder, sub_folder, msg_type), topic_name

  // create subscribers
  RCCHECK(rclc_subscription_init_default(
    &sub1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros"));

  RCCHECK(rclc_subscription_init_default(
    &sub2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "cmd_vel"));

  // create timer,
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
            &timer,
            &support,
            RCL_MS_TO_NS(timer_timeout),
            timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub1, &msg_sub1, &subscription_callback_1, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub2, &msg_sub2, &subscription_callback_2, ON_NEW_DATA));

  msg_pub1.data = 0;   //parameter data of Int32 msg
  msg_pub2.data = 0;   //parameter data of Int32 msg
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
