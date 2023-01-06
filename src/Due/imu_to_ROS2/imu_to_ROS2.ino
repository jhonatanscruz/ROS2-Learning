#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <Wire.h>

// Publishers
rcl_publisher_t imu_pub;
sensor_msgs__msg__Imu imu_msg; //

//Auxiliares
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

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
    RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
    //msg_pub1.data++;
  }
}

void initMPU6050(const int Adress = 0x68){
    // Inicializa o MPU-6050
    Wire.begin();
    Wire.beginTransmission(Adress);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);

    // Configura Giroscópio para fundo de escala desejado
    /*
    Wire.write(0b00000000); // fundo de escala em +/-250°/s
    Wire.write(0b00001000); // fundo de escala em +/-500°/s
    Wire.write(0b00010000); // fundo de escala em +/-1000°/s
    Wire.write(0b00011000); // fundo de escala em +/-2000°/s
    */
    Wire.beginTransmission(Adress);
    Wire.write(0x1B);
    Wire.write(0b00011000);  // Trocar esse comando para fundo de escala desejado conforme acima
    Wire.endTransmission();

    // Configura Acelerometro para fundo de escala desejado
    /*
      Wire.write(0b00000000); // fundo de escala em +/-2g
      Wire.write(0b00001000); // fundo de escala em +/-4g
      Wire.write(0b00010000); // fundo de escala em +/-8g
      Wire.write(0b00011000); // fundo de escala em +/-16g
    */
    Wire.beginTransmission(Adress);
    Wire.write(0x1C);
    Wire.write(0b00011000);  // Trocar esse comando para fundo de escala desejado conforme acima
    Wire.endTransmission();
}

float* readMPU6050(const int Adress = 0x68){

    float MPU_msg[7];
    // Variaveis para armazenar valores do sensor
    int16_t AccX, AccY, AccZ, Temp, GyrX, GyrY, GyrZ;

    // Comandos para iniciar transmissão de dados
    Wire.beginTransmission(Adress);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(Adress, 14, true); // Solicita os dados ao sensor

    // Armazena o valor dos sensores nas variaveis correspondentes
    AccX = Wire.read() << 8 | Wire.read(); //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    AccY = Wire.read() << 8 | Wire.read(); //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AccZ = Wire.read() << 8 | Wire.read(); //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Temp = Wire.read() << 8 | Wire.read(); //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyrX = Wire.read() << 8 | Wire.read(); //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyrY = Wire.read() << 8 | Wire.read(); //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyrZ = Wire.read() << 8 | Wire.read(); //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    /* Alterar divisão conforme fundo de escala escolhido:
      Acelerômetro
      +/-2g = 16384
      +/-4g = 8192
      +/-8g = 4096
      +/-16g = 2048
    
      Giroscópio
      +/-250°/s = 131
      +/-500°/s = 65.6
      +/-1000°/s = 32.8
      +/-2000°/s = 16.4
    */

    MPU_msg[0] = float(AccX / 2048.);
    MPU_msg[1] = float(AccY / 2048.);
    MPU_msg[2] = float(AccZ / 2048.);
    MPU_msg[3] = float(Temp);
    MPU_msg[4] = float(GyrX / 16.4);
    MPU_msg[5] = float(GyrY / 16.4);
    MPU_msg[6] = float(GyrZ / 16.4);

    return MPU_msg;
}

/*
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
            &imu_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
            "imu/data")); //(parent_folder, sub_folder, msg_type), topic_name

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

  //msg_pub1.data = 0;   //parameter data of Int32 msg
  //msg_pub2.data = 0;   //parameter data of Int32 msg

}

void loop() {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
*/
float* MPU_msg;

void setup() {
    Serial.begin(9600);
    initMPU6050();
}

void loop() {
    MPU_msg = readMPU6050();
    Serial.println("AccX = " + String(MPU_msg[0]));
    Serial.println("AccY = " + String(MPU_msg[1]));
    Serial.println("AccZ = " + String(MPU_msg[2]));
    Serial.println("GyrX = " + String(MPU_msg[4]));
    Serial.println("GyrY = " + String(MPU_msg[5]));
    Serial.println("GyrZ = " + String(MPU_msg[6]));
    delay(2000);
}
