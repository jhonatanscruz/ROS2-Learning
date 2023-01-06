/*
 * Author: Jhonatan Cruz (jhonatan@fttech.com.br)
 * Maintener: FTTech Developer team (contato@fttech.com.br)
 * 
 * Description: ROS2 node that publishes the accumulated ticks for each robot wheel
 * (/front_right_ticks /front_left_ticks /rear_right_ticks and /rear_left_ticks topics)
 * at regular intervals using the built-in encoder (forward = positive; reverse = negative). 
 * The node also subscribes to linear & angular velocity commands published on 
 * the /cmd_vel topic to drive the robot accordingly.
 * 
 * References: [1] Practical Robotics in C++ book (ISBN-10 : 9389423465)
 *             [2] Automatic Addison code (https://automaticaddison.com)
*/

#include <FTTech_Components.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>

////////////////// Tick Data Publishing/Motor Controller Constants ///////////////

// Motor/Encoder Set 1
#define PWM1 12  // Motor_1 Velocity
#define DIR1A 34 // Motor_1 Direction pulse A
#define DIR1B 35 // Motor_1 Direction pulse B
#define EN1_A 18 // Encoder_1 pulse A
#define EN1_B 31 // Encoder_1 pulse B

// Motor/Encoder Set 2
#define PWM2 8   // Motor_2 Velocity
#define DIR2A 36 // Motor_2 Direction pulse A
#define DIR2B 37 // Motor_2 Direction pulse B
#define EN2_A 38 // Encoder_2 pulse A
#define EN2_B 19 // Encoder_2 pulse B

// Motor/Encoder Set 3 connections
#define PWM3 9   // Motor_3 Velocity
#define DIR3A 43 // Motor_3 Direction pulse A
#define DIR3B 42 // Motor_3 Direction pulse B
#define EN3_A 3 // Encoder_3 pulse A
#define EN3_B 49  // Encoder_3 pulse B

// Motor/Encoder Set 4 connections
#define PWM4 5   // Motor_4 Velocity
#define DIR4A A5 // Motor_4 Direction pulse A
#define DIR4B A4 // Motor_4 Direction pulse B
#define EN4_A A1  // Encoder_4 pulse A
#define EN4_B 2 // Encoder_4 pulse B

/*
// MOTOR RUNNING

#define MOTOR1_RUN(pwm) do{if(pwm > 0){digitalWrite(DIR1A,LOW);digitalWrite(DIR1B,HIGH);analogWrite(PWM1,pwm);} else{digitalWrite(DIR1A,HIGH);digitalWrite(DIR1B,LOW);analogWrite(PWM1,-pwm);}}while(0)
#define MOTOR1_STOP(x)  do{digitalWrite(DIR1A,LOW); digitalWrite(DIR1B,LOW); analogWrite(PWM1,0);}while(0)

#define MOTOR2_RUN(pwm) do{if(pwm > 0){digitalWrite(DIR2A,LOW);digitalWrite(DIR2B,HIGH);analogWrite(PWM2,pwm);} else{digitalWrite(DIR2A,HIGH);digitalWrite(DIR2B,LOW);analogWrite(PWM2,-pwm);}}while(0)
#define MOTOR2_STOP(x)  do{digitalWrite(DIR2A,LOW); digitalWrite(DIR2B,LOW); analogWrite(PWM2,0);}while(0)

#define MOTOR3_RUN(pwm) do{if(pwm > 0){digitalWrite(DIR3A,LOW);digitalWrite(DIR3B,HIGH);analogWrite(PWM3,pwm);} else{digitalWrite(DIR3A,HIGH);digitalWrite(DIR3B,LOW);analogWrite(PWM3,-pwm);}}while(0)
#define MOTOR3_STOP(x)  do{digitalWrite(DIR3A,LOW); digitalWrite(DIR3B,LOW); analogWrite(PWM3,0);}while(0)

#define MOTOR4_RUN(pwm) do{if(pwm > 0){digitalWrite(DIR4A,LOW);digitalWrite(DIR4B,HIGH);analogWrite(PWM4,pwm);} else{digitalWrite(DIR4A,HIGH);digitalWrite(DIR4B,LOW);analogWrite(PWM4,-pwm);}}while(0)
#define MOTOR4_STOP(x)  do{digitalWrite(DIR4A,LOW); digitalWrite(DIR4B,LOW); analogWrite(PWM4,0);}while(0)
*/

// Define as classes para cada um dos encoders
FT_Encoder2 encoder1(EN1_A, EN1_B);
FT_Encoder2 encoder2(EN2_A, EN2_B);
FT_Encoder2 encoder3(EN3_A, EN3_B);
FT_Encoder2 encoder4(EN4_A, EN4_B);

// Especial Time Function
extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);

////////////////// Tick Data Publishing/Motor Controller variables ///////////////

// Minumum and maximum values for 32-bit integers
// Range of 4294967296
const long int encoder_minimum = -2147483648;
const long int encoder_maximum = 2147483647;

// Time interval for measurements in milliseconds
const int interval = 30;
long previousMillis = 0;
long currentMillis = 0;

// How much the PWM value can change each cycle
const int PWM_INCREMENT = 1;

// Number of ticks per wheel revolution. We won't use this in this code.
const int TICKS_PER_REVOLUTION = 1340;

// Wheel radius in meters
const double WHEEL_RADIUS = 0.04;

// Distance from center of the left tire to the center of the right tire in m
const double WHEEL_BASE = 0.2;

// Number of ticks a wheel makes moving a linear distance of 1 meter
// This value was measured manually.
const double TICKS_PER_METER = 5330;
 
// Proportional constant, which was measured by measuring the
// PWM-Linear Velocity relationship for the robot.

// PWM_Value = (278/sin(45)) * (m/s speed) + 52

const int K_P = 393;

// Y-intercept for the PWM-Linear Velocity relationship for the robot
const int b = 52;

// Correction multiplier for drift. Chosen through experimentation.
const int DRIFT_MULTIPLIER = 120;

// Set maximum and minimum limits for the PWM values
const int PWM_MIN = 70; // about 0.065 m/s
const int PWM_MAX = 100; // about 0.172 m/s

// Turning PWM output (0 = min, 255 = max for PWM values)
const int PWM_TURN = PWM_MIN;

// Set linear velocity and PWM variable values for each wheel
double velLeftFrontWheel = 0;
double velRightFrontWheel = 0;
double velLeftRearWheel = 0;
double velRightRearWheel = 0;
double pwmLeftFrontReq = 0;
double pwmRightFrontReq = 0;
double pwmLeftRearReq = 0;
double pwmRightRearReq = 0;

// Robots dynamics
double robot_vel_x = 0;
double robot_vel_y = 0;
double vel_x = 0;
double vel_y = 0;
double vel_yaw = 0;
double pose_x = 0;
double pose_y = 0;
double yaw = 0;

// Wheel Factors
double lf_Hfactor = 0;
double rf_Hfactor = 0;
double lr_Hfactor = 0;
double rr_Hfactor = 0;
double lf_Vfactor = 0;
double rf_Vfactor = 0;
double lr_Vfactor = 0;
double rr_Vfactor = 0;

// Initial pose
double initialX = 0.00000000001;
double initialY = 0.00000000001;
double initialYaw = 0.00000000001;

// Record the time that the last velocity command was received
double lastCmdVelReceived = 0;

////////////////// ROS2 environment Variables and Constants ///////////////

// Publishers
rcl_publisher_t pub_left_ticks;     // Left Ticks Publisher
std_msgs__msg__Int32 msg_left_ticks;  // Left Ticks Message

rcl_publisher_t pub_right_ticks;       // Right Ticks Publisher
std_msgs__msg__Int32 msg_right_ticks;  // Right Ticks Message

/*
rcl_publisher_t pub_lr_ticks;       // Left Rear Ticks Publisher
std_msgs__msg__Int32 msg_lr_ticks;  // Left Rear Ticks Message

rcl_publisher_t pub_rr_ticks;       // Right Rear Ticks Publisher
std_msgs__msg__Int32 msg_rr_ticks;  // Right Rear Ticks Message


rcl_publisher_t pub_odom;           // Odometry Publisher
nav_msgs__msg__Odometry msg_odom;   // Odometry Message
*/

// Subscribers
rcl_subscription_t sub_cmd_vel;     // Cmd_Vel Subscriber
geometry_msgs__msg__Twist msg_cmd_vel;   // Cmd_Vel Message

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

/////////////////////// General Purpose Functions ////////////////////////////

const void euler_to_quat(double x, double y, double z, double* q) {
    double c1 = cos(y/2);
    double c2 = cos(z/2);
    double c3 = cos(x/2);

    double s1 = sin(y/2);
    double s2 = sin(z/2);
    double s3 = sin(x/2);

    q[0] = c1 * c2 * c3 - s1 * s2 * s3;
    q[1] = s1 * s2 * c3 + c1 * c2 * s3;
    q[2] = s1 * c2 * c3 + c1 * s2 * s3;
    q[3] = c1 * s2 * c3 - s1 * c2 * s3;
}

/////////////////////// Motor Controller Functions ////////////////////////////

// Calculate the left front wheel linear velocity in m/s every time a 
// tick count message is published on the wheel/left_front_ticks topic. 
void calc_vel_lf_wheel(){

  static double prev_lf_time = 0;
  static int prev_lf_count = 0;
  volatile int long lf_ticks = encoder1.getPosition();

  // Manage rollover and rollunder when we get outside the 32-bit integer range 
  int numOfTicks = lf_ticks - prev_lf_count;

  // Calculate wheel velocity in meters per second
  velLeftFrontWheel = numOfTicks/TICKS_PER_METER/((millis()/1000)- prev_lf_time);

  // Keep track of the previous tick count
  prev_lf_count = lf_ticks;
 
  // Update the timestamp
  prev_lf_time = (millis()/1000);
}

// ###############################################################################################

// Calculate the right front wheel linear velocity in m/s every time a 
// tick count message is published on the wheel/right_front_ticks topic. 
void calc_vel_rf_wheel(){

    static double prev_rf_time = 0;
    static int prev_rf_count = 0;
    volatile int long rf_ticks = encoder2.getPosition();

    // Manage rollover and rollunder when we get outside the 32-bit integer range 
    int numOfTicks = rf_ticks - prev_rf_count;

    // Calculate wheel velocity in meters per second
    velRightFrontWheel = numOfTicks/TICKS_PER_METER/((millis()/1000)- prev_rf_time);

    // Keep track of the previous tick count
    prev_rf_count = rf_ticks;

    // Update the timestamp
    prev_rf_time = (millis()/1000);
}

// ###############################################################################################

// Calculate the left rear wheel linear velocity in m/s every time a 
// tick count message is published on the wheel/left_rear_ticks topic. 
void calc_vel_lr_wheel(){

  static double prev_lr_time = 0;
  static int prev_lr_count = 0;
  volatile int long lr_ticks = encoder3.getPosition();

  // Manage rollover and rollunder when we get outside the 32-bit integer range 
  int numOfTicks = lr_ticks - prev_lr_count;

  // Calculate wheel velocity in meters per second
  velLeftRearWheel = numOfTicks/TICKS_PER_METER/((millis()/1000)- prev_lr_time);
 
  // Keep track of the previous tick count
  prev_lr_count = lr_ticks;
 
  // Update the timestamp
  prev_lr_time = (millis()/1000);
}

// ###############################################################################################

// Calculate the left rear wheel linear velocity in m/s every time a 
// tick count message is published on the wheel/right_rear_ticks topic. 
void calc_vel_rr_wheel(){

  static double prev_rr_time = 0;
  static int prev_rr_count = 0;
  volatile int long rr_ticks = encoder4.getPosition();

  // Manage rollover and rollunder when we get outside the 32-bit integer range
  int numOfTicks = rr_ticks - prev_rr_count;

  // Calculate wheel velocity in meters per second
  velRightRearWheel = numOfTicks/TICKS_PER_METER/((millis()/1000)- prev_rr_time);

  // Keep track of the previous tick count
  prev_rr_count = rr_ticks;

  // Update the timestamp
  prev_rr_time = (millis()/1000);
}

// ###############################################################################################

// Take the velocity command as input and calculate the PWM values.
void calc_pwm_values(const geometry_msgs__msg__Twist &cmdVel){

    static double prevFrontDiff = 0;
    static double prevPrevFrontDiff = 0;
    static double prevRearDiff = 0;
    static double prevPrevRearDiff = 0;

    // Record timestamp of last velocity command received
    lastCmdVelReceived = (millis()/1000);

    // Calculate the PWM value given the desired velocity

    if(cmdVel.linear.x > 0){
        pwmLeftFrontReq = (K_P * cmdVel.linear.x) + b;
        pwmRightFrontReq = (K_P * cmdVel.linear.x) + b;
        pwmLeftRearReq = (K_P * cmdVel.linear.x) + b;
        pwmRightRearReq = (K_P * cmdVel.linear.x) + b;
    }

    else{
        pwmLeftFrontReq = (K_P * cmdVel.linear.x) - b;
        pwmRightFrontReq = (K_P * cmdVel.linear.x) - b;
        pwmLeftRearReq = (K_P * cmdVel.linear.x) - b;
        pwmRightRearReq = (K_P * cmdVel.linear.x) - b;
    }


  // Check if we need to turn 
  if (cmdVel.angular.z != 0.0) {

    // Turn left
    if (cmdVel.angular.z > 0.0) {
      pwmLeftFrontReq = -PWM_TURN;
      pwmLeftRearReq = -PWM_TURN;
      pwmRightFrontReq = PWM_TURN;
      pwmRightRearReq = PWM_TURN;
    }
    // Turn right    
    else {
      pwmLeftFrontReq = PWM_TURN;
      pwmLeftRearReq = PWM_TURN;
      pwmRightFrontReq = -PWM_TURN;
      pwmRightRearReq = -PWM_TURN;
    }
  }
  // Go straight
  else {
    /*
    // Remove any differences in wheel velocities to make sure the robot goes straight
    double currFrontDifference = velLeftFrontWheel - velRightFrontWheel;
    double currRearDifference = velLeftRearWheel - velRightRearWheel;
    double avgFrontDifference = (prevFrontDiff+prevPrevFrontDiff+currFrontDifference)/3;
    double avgRearDifference = (prevRearDiff+prevPrevRearDiff+currRearDifference)/3;

    prevPrevFrontDiff = prevFrontDiff;
    prevFrontDiff = currFrontDifference;
    prevPrevRearDiff = prevRearDiff;
    prevRearDiff = currRearDifference;

    // Correct PWM values of each wheel to make the vehicle go straight
    pwmLeftFrontReq -= (int)(avgFrontDifference * DRIFT_MULTIPLIER);
    pwmRightFrontReq += (int)(avgFrontDifference * DRIFT_MULTIPLIER);
    pwmLeftRearReq -= (int)(avgRearDifference * DRIFT_MULTIPLIER);
    pwmRightRearReq += (int)(avgRearDifference * DRIFT_MULTIPLIER);
    */
  }

  // Handle low PWM values
  if (abs(pwmLeftFrontReq) < PWM_MIN) {
    pwmLeftFrontReq = 0;
  }
  if (abs(pwmRightFrontReq) < PWM_MIN) {
    pwmRightFrontReq = 0;  
  }
  if (abs(pwmLeftRearReq) < PWM_MIN) {
    pwmLeftRearReq = 0;  
  }
  if (abs(pwmRightRearReq) < PWM_MIN) {
    pwmRightRearReq = 0;  
  }

}

// ###############################################################################################

void set_pwm_values() {

    // These variables will hold our desired PWM values
    static int pwmLFOut = 0;
    static int pwmRFOut = 0;
    static int pwmLROut = 0;
    static int pwmRROut = 0;

    // Set the direction of the motors
    // ******************* LEFT FRONT WHEEL *******************
    if (pwmLeftFrontReq > 0) { // LF wheel forward
        digitalWrite(DIR1A, LOW);
        digitalWrite(DIR1B, HIGH);
        lf_Hfactor = cos(135*PI/180);
        lf_Vfactor = sin(135*PI/180);
    }

    else if (pwmLeftFrontReq < 0) { // LF wheel reverse
        digitalWrite(DIR1A, HIGH);
        digitalWrite(DIR1B, LOW);
        lf_Hfactor = cos(315*PI/180);
        lf_Vfactor = sin(315*PI/180);
    }

    else { // LF wheel stops
        digitalWrite(DIR1A, LOW);
        digitalWrite(DIR1B, LOW);
        lf_Hfactor = 0;
        lf_Vfactor = 0;
    }

    // ******************* RIGHT FRONT WHEEL *******************
    if (pwmRightFrontReq > 0) { // RF wheel forward
        digitalWrite(DIR2A, LOW);
        digitalWrite(DIR2B, HIGH);
        rf_Hfactor = cos(45*PI/180);
        rf_Vfactor = sin(45*PI/180);
    }

    else if(pwmRightFrontReq < 0) { // RF wheel reverse
        digitalWrite(DIR2A, HIGH);
        digitalWrite(DIR2B, LOW);
        rf_Hfactor = cos(225*PI/180);
        rf_Vfactor = sin(225*PI/180);
    }

    else { // RF wheel stops
        digitalWrite(DIR2A, LOW);
        digitalWrite(DIR2B, LOW);
        rf_Hfactor = 0;
        rf_Vfactor = 0;
    }

    // ******************* LEFT REAR WHEEL *******************
    if (pwmLeftRearReq > 0) { // LR wheel forward
        digitalWrite(DIR3A, LOW);
        digitalWrite(DIR3B, HIGH);
        lr_Hfactor = cos(45*PI/180);
        lr_Vfactor = sin(45*PI/180);
    }

    else if (pwmLeftRearReq < 0) { // LR wheel reverse
        digitalWrite(DIR3A, HIGH);
        digitalWrite(DIR3B, LOW);
        lr_Hfactor = cos(225*PI/180);
        lr_Vfactor = sin(315*PI/180);
    }

    else { // LR wheel stops
        digitalWrite(DIR3A, LOW);
        digitalWrite(DIR3B, LOW);
        lr_Hfactor = 0;
        lr_Vfactor = 0;
    }

    // ******************* RIGHT REAR WHEEL *******************
    if (pwmRightRearReq > 0) { // RR wheel forward
        digitalWrite(DIR4A, LOW);
        digitalWrite(DIR4B, HIGH);
        rr_Hfactor = cos(135*PI/180);
        rr_Vfactor = sin(135*PI/180);
    }

    else if(pwmRightRearReq < 0) { // RR wheel reverse
        digitalWrite(DIR4A, HIGH);
        digitalWrite(DIR4B, LOW);
        rr_Hfactor = cos(315*PI/180);
        rr_Vfactor = sin(315*PI/180);
    }

    else { // RR wheel stops
        digitalWrite(DIR4A, LOW);
        digitalWrite(DIR4B, LOW);
        rr_Hfactor = 0;
        rr_Vfactor = 0;
    }

    // Calculate the output PWM value by making slow changes to the current value
    // ******************* LEFT FRONT WHEEL *******************
    if (abs(pwmLeftFrontReq) > pwmLFOut) {
        pwmLFOut += PWM_INCREMENT;
    }

    else if (abs(pwmLeftFrontReq) < pwmLFOut) {
        pwmLFOut -= PWM_INCREMENT;
    }

    else{}

    // ******************* RIGHT FRONT WHEEL *******************
    if (abs(pwmRightFrontReq) > pwmRFOut) {
        pwmRFOut += PWM_INCREMENT;
    }

    else if (abs(pwmRightFrontReq) < pwmRFOut) {
        pwmRFOut -= PWM_INCREMENT;
    }

    else{}

    // ******************* LEFT REAR WHEEL *******************
    if (abs(pwmLeftRearReq) > pwmLROut) {
        pwmLROut += PWM_INCREMENT;
    }

    else if (abs(pwmLeftRearReq) < pwmLROut) {
        pwmLROut -= PWM_INCREMENT;
    }

    else{}

    // ******************* RIGHT REAR WHEEL *******************
    if (abs(pwmRightRearReq) > pwmRROut) {
        pwmRROut += PWM_INCREMENT;
    }

    else if (abs(pwmRightRearReq) < pwmRROut) {
        pwmRROut -= PWM_INCREMENT;
    }

    else{}

    // Conditional operator to limit PWM output at the maximum 
    pwmLFOut = (pwmLFOut > PWM_MAX) ? PWM_MAX : pwmLFOut;
    pwmRFOut = (pwmRFOut > PWM_MAX) ? PWM_MAX : pwmRFOut;
    pwmLROut = (pwmLROut > PWM_MAX) ? PWM_MAX : pwmLROut;
    pwmRROut = (pwmRROut > PWM_MAX) ? PWM_MAX : pwmRROut;

    // PWM output cannot be less than 0
    pwmLFOut = (pwmLFOut < 0) ? 0 : pwmLFOut;
    pwmRFOut = (pwmRFOut < 0) ? 0 : pwmRFOut;
    pwmLROut = (pwmLROut < 0) ? 0 : pwmLROut;
    pwmRROut = (pwmRROut < 0) ? 0 : pwmRROut;

    // Set the PWM value on the pins
    analogWrite(PWM1, pwmLFOut);
    analogWrite(PWM2, pwmRFOut);
    analogWrite(PWM3, pwmLROut);
    analogWrite(PWM4, pwmRROut);
}
// ###############################################################################################
/*
/////////////////////// Robot Dynamics Function ////////////////////////////
void dynamics(){

    static double prev_pose_x = 0.0;
    static double prev_pose_y = 0.0;
    static double prev_yaw = 0.0;
    static double prev_dynamics_time = 0;
    static double prev_right_count = 0;
    static double prev_left_count = 0;

    // Keep the mean delta ticks of each robot side
    double delta_right_ticks = ((encoder2.getPosition() + encoder4.getPosition())/2) - prev_right_count;
    double delta_left_ticks =  ((encoder1.getPosition() + encoder3.getPosition())/2) - prev_left_count;

    // Robot World pose
    pose_x = prev_pose_x + (vel_x*((millis()/1000) - prev_dynamics_time));
    pose_y = prev_pose_y + (vel_y*((millis()/1000) - prev_dynamics_time));

    // Orientation
    yaw = asin((delta_right_ticks - delta_left_ticks) / TICKS_PER_METER / WHEEL_BASE) + prev_yaw;

    if (yaw > PI) yaw -= 2*PI;
    else if (yaw < -PI) yaw += 2*PI;
    else{}

    vel_yaw = (yaw - prev_yaw) / ((millis()/1000) - prev_dynamics_time);

    // Wheel velocity calc
    calc_vel_lf_wheel();
    calc_vel_rf_wheel();
    calc_vel_lr_wheel();
    calc_vel_rr_wheel();

    // Change previous values
    prev_dynamics_time = (millis()/1000);
    prev_pose_x = pose_x;
    prev_pose_y = pose_y;
    prev_yaw = yaw;
    prev_right_count = delta_right_ticks;
    prev_left_count =  delta_left_ticks;

    // Robot Local velocity
    robot_vel_x = (velLeftFrontWheel*lf_Hfactor + velRightFrontWheel*rf_Hfactor + velLeftRearWheel*lr_Hfactor + velRightRearWheel*rr_Hfactor)/4;
    robot_vel_y = (velLeftFrontWheel*lf_Vfactor + velRightFrontWheel*rf_Vfactor + velLeftRearWheel*lr_Vfactor + velRightRearWheel*rr_Vfactor)/4;

    // Robot World velocity
    vel_x = robot_vel_x*cos(yaw) - robot_vel_y*sin(yaw);
    vel_y = robot_vel_x*sin(yaw) + robot_vel_y*cos(yaw);
}
*/
// ###############################################################################################

/////////////////////// ROS2 Environment Functions ////////////////////////////
void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// ###############################################################################################

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {

        msg_left_ticks.data = (encoder1.getPosition() + encoder3.getPosition())/2;
        msg_right_ticks.data = (encoder2.getPosition() + encoder4.getPosition())/2;

/*
        double q[4];

        struct timespec tv = {0};
        clock_gettime(0, &tv);

        // #### It processes all dynamic robot calculations ###
        dynamics();

        // ##### Feed Odometry msg #####
        msg_odom.header.stamp.sec = tv.tv_sec;
        msg_odom.header.stamp.nanosec = tv.tv_nsec;
        msg_odom.header.frame_id.data = "odom";
        msg_odom.child_frame_id.data = "base_link";

        msg_odom.pose.pose.position.x = pose_x;
        msg_odom.pose.pose.position.y = pose_y;
        msg_odom.pose.pose.position.z = 0;

        euler_to_quat(0, 0, yaw, q);
        msg_odom.pose.pose.orientation.x = (double) q[1];
        msg_odom.pose.pose.orientation.y = (double) q[2];
        msg_odom.pose.pose.orientation.z = (double) q[3];
        msg_odom.pose.pose.orientation.w = (double) q[0];

        msg_odom.twist.twist.linear.x = vel_x;
        msg_odom.twist.twist.linear.y = vel_y;
        msg_odom.twist.twist.linear.z = 0;
        msg_odom.twist.twist.angular.z = vel_yaw;

        for(int i = 0; i<36; i++) {
            if(i == 0 || i == 7 || i == 14) {
                msg_odom.pose.covariance[i] = .01;
            }
            else if (i == 21 || i == 28 || i== 35) {
                msg_odom.pose.covariance[i] += 0.1;
            }
            else {
                msg_odom.pose.covariance[i] = 0;
            }
        }
*/
        // ##### Publish Messages Over ROS2 #####
        //RCSOFTCHECK(rcl_publish(&pub_odom, &msg_odom, NULL));
        RCSOFTCHECK(rcl_publish(&pub_left_ticks, &msg_left_ticks, NULL));
        RCSOFTCHECK(rcl_publish(&pub_right_ticks, &msg_right_ticks, NULL));
    }
}

// ###############################################################################################

void subscription_callback_1(const void * msgin)
{
    const geometry_msgs__msg__Twist * cmdVel = (const geometry_msgs__msg__Twist *)msgin;
    calc_pwm_values(*cmdVel);
}

// ###############################################################################################

void subscription_callback_2(const void * msgin)
{
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    digitalWrite(LED_PIN, (msg->data == -1) ? !(digitalRead(LED_PIN)) : (digitalRead(LED_PIN)));  
}

// ###############################################################################################

/**********************************
              SETUP
**********************************/
void setup() {

    // Turn off motors - Initial state
    digitalWrite(DIR1A, LOW);
    digitalWrite(DIR1B, LOW);
    digitalWrite(DIR2A, LOW);
    digitalWrite(DIR2B, LOW);
    digitalWrite(DIR3A, LOW);
    digitalWrite(DIR3B, LOW);
    digitalWrite(DIR4A, LOW);
    digitalWrite(DIR4B, LOW);

    // Set the motor speed 0
    analogWrite(PWM1, 0); 
    analogWrite(PWM2, 0);
    analogWrite(PWM3, 0); 
    analogWrite(PWM4, 0);

    // Encoders reset
    encoder1.begin();
    encoder2.begin();
    encoder3.begin();
    encoder4.begin();

    set_microros_transports();

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    allocator = rcl_get_default_allocator();

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "arduino", "", &support));

    // create publishers [(parent_folder, sub_folder, msg_type), topic_name]
    RCCHECK(rclc_publisher_init_default(&pub_left_ticks,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),"left_ticks"));
    RCCHECK(rclc_publisher_init_default(&pub_right_ticks,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),"right_ticks"));

    // create subscribers
    RCCHECK(rclc_subscription_init_default(&sub_cmd_vel,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),"/cmd_vel"));
    RCCHECK(rclc_subscription_init_default(&sub2,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),"/goal"));

    // create timer,
    const unsigned int timer_timeout = 100;
    RCCHECK(rclc_timer_init_default(&timer,&support,RCL_MS_TO_NS(timer_timeout),timer_callback));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel, &subscription_callback_1, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub2, &msg_sub2, &subscription_callback_2, ON_NEW_DATA));

    // INTERRUPTIONS
    // ******************* LEFT FRONT ENCODER *******************
    attachInterrupt(digitalPinToInterrupt(EN1_A), encoder1A_OnChange, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EN1_B), encoder1B_OnChange, CHANGE);
    // ******************* RIGHT FRONT ENCODER *******************
    attachInterrupt(digitalPinToInterrupt(EN2_A), encoder2A_OnChange, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EN2_B), encoder2B_OnChange, CHANGE);
    // ******************* LEFT REAR ENCODER *******************
    attachInterrupt(digitalPinToInterrupt(EN3_A), encoder3A_OnChange, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EN3_B), encoder3B_OnChange, CHANGE);
    // ******************* RIGHT REAR ENCODER *******************
    attachInterrupt(digitalPinToInterrupt(EN4_A), encoder4A_OnChange, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EN4_B), encoder4B_OnChange, CHANGE);
}

// ###############################################################################################

void loop() {

    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

    // Stop the car if there are no cmd_vel messages
    if((millis()/1000) - lastCmdVelReceived > 0.11) { // It breaks the robot if any /cmd_vel message is received after 110ms
        pwmLeftFrontReq = 0;
        pwmRightFrontReq = 0;
        pwmLeftRearReq = 0;
        pwmRightRearReq = 0;
    }

    set_pwm_values();
}

// ###############################################################################################

// -------------------- GLOBAL FUNCTIONS --------------------

// ******************* LEFT FRONT ENCODER *******************
void encoder1A_OnChange(){
  encoder1.PinA_OnChange();
}
// ---------------------------

void encoder1B_OnChange(){
  encoder1.PinB_OnChange();
}

// ******************* RIGHT FRONT ENCODER *******************
void encoder2A_OnChange(){
  encoder2.PinA_OnChange();
}
// ---------------------------

void encoder2B_OnChange(){
  encoder2.PinB_OnChange();
}
// ******************* LEFT REAR ENCODER *******************
void encoder3A_OnChange(){
  encoder3.PinA_OnChange();
}
// ---------------------------

void encoder3B_OnChange(){
  encoder3.PinB_OnChange();
}
// ******************* RIGHT REAR ENCODER *******************
void encoder4A_OnChange(){
  encoder4.PinA_OnChange();
}
// ---------------------------

void encoder4B_OnChange(){
  encoder4.PinB_OnChange();
}
// ---------------------------
