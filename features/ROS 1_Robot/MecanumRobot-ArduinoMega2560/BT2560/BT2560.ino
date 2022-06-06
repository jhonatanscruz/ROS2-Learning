#include <Wire.h>
#include <string.h>

String received_msg = "";

//Pinos Motor
#define PWMA 12    //A - Velocidade
#define DIRA1 34 
#define DIRA2 35  //A电机方向

#define PWMB 8    //B - Velocidade
#define DIRB1 36 
#define DIRB2 37  //B电机方向

#define PWMC 9   //C - Velocidade
#define DIRC1 43 
#define DIRC2 42  //C电机方向

#define PWMD 5    //D - Velocidade
#define DIRD1 A5  
#define DIRD2 A4  //D电机方向

#define MOTORA_FORWARD(pwm)    do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,HIGH);analogWrite(PWMA,pwm);}while(0)
#define MOTORA_STOP(x)         do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,LOW); analogWrite(PWMA,0);}while(0)
#define MOTORA_BACKOFF(pwm)    do{digitalWrite(DIRA1,HIGH);digitalWrite(DIRA2,LOW); analogWrite(PWMA,pwm);}while(0)

#define MOTORB_FORWARD(pwm)    do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,HIGH);analogWrite(PWMB,pwm);}while(0)
#define MOTORB_STOP(x)         do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,LOW); analogWrite(PWMB,0);}while(0)
#define MOTORB_BACKOFF(pwm)    do{digitalWrite(DIRB1,HIGH);digitalWrite(DIRB2,LOW); analogWrite(PWMB,pwm);}while(0)

#define MOTORC_FORWARD(pwm)    do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,HIGH);analogWrite(PWMC,pwm);}while(0)
#define MOTORC_STOP(x)         do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,LOW); analogWrite(PWMC,0);}while(0)
#define MOTORC_BACKOFF(pwm)    do{digitalWrite(DIRC1,HIGH);digitalWrite(DIRC2,LOW); analogWrite(PWMC,pwm);}while(0)

#define MOTORD_FORWARD(pwm)    do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,HIGH);analogWrite(PWMD,pwm);}while(0)
#define MOTORD_STOP(x)         do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,LOW); analogWrite(PWMD,0);}while(0)
#define MOTORD_BACKOFF(pwm)    do{digitalWrite(DIRD1,HIGH);digitalWrite(DIRD2,LOW); analogWrite(PWMD,pwm);}while(0)

#define SERIAL  Serial

#define LOG_DEBUG

#ifdef LOG_DEBUG
#define M_LOG SERIAL.print
#else
#define M_LOG 
#endif

#define MAX_PWM   200
#define MIN_PWM   130
int Motor_PWM = 75;

//控制电机运动    宏定义

//    ↑A-----B↑   
//     |  ↑  |
//     |  |  |
//    ↑C-----D↑
void ADVANCE()
{
  MOTORA_FORWARD(Motor_PWM);MOTORB_FORWARD(Motor_PWM);    
  MOTORC_FORWARD(Motor_PWM);MOTORD_FORWARD(Motor_PWM);    
}

// --------------------------------------------------------

//    ↓A-----B↓   
//     |  |  |
//     |  ↓  |
//    ↓C-----D↓
void BACK()
{
  MOTORA_BACKOFF(Motor_PWM);MOTORB_BACKOFF(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);MOTORD_BACKOFF(Motor_PWM);
}

// --------------------------------------------------------


//    =A-----B↑   
//     |   ↖ |
//     | ↖   |
//    ↑C-----D=
void LEFT_1()
{
  MOTORA_STOP(Motor_PWM);MOTORB_FORWARD(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);MOTORD_STOP(Motor_PWM);
}

// --------------------------------------------------------

//    ↓A-----B↑   
//     |  ←  |
//     |  ←  |
//    ↑C-----D↓
void LEFT_2()
{
  MOTORA_BACKOFF(Motor_PWM);MOTORB_FORWARD(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);MOTORD_BACKOFF(Motor_PWM);
}

// --------------------------------------------------------


//    ↓A-----B=   
//     | ↙   |
//     |   ↙ |
//    =C-----D↓
void LEFT_3()
{
  MOTORA_BACKOFF(Motor_PWM);MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);MOTORD_BACKOFF(Motor_PWM);
}

// --------------------------------------------------------


//    ↑A-----B=   
//     | ↗   |
//     |   ↗ |
//    =C-----D↑
void RIGHT_1()
{
  MOTORA_FORWARD(Motor_PWM);MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);MOTORD_FORWARD(Motor_PWM);
}

// --------------------------------------------------------


//    ↑A-----B↓   
//     |  →  |
//     |  →  |
//    ↓C-----D↑
void RIGHT_2()
{
  MOTORA_FORWARD(Motor_PWM);MOTORB_BACKOFF(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);MOTORD_FORWARD(Motor_PWM);
}

// --------------------------------------------------------

//    =A-----B↓   
//     |   ↘ |
//     | ↘   |
//    ↓C-----D=
void RIGHT_3()
{
  MOTORA_STOP(Motor_PWM);MOTORB_BACKOFF(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);MOTORD_STOP(Motor_PWM);
}

// --------------------------------------------------------


//    =A-----B=  
//     |  =  |
//     |  =  |
//    =C-----D=
void STOP()
{
  MOTORA_STOP(Motor_PWM);MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);MOTORD_STOP(Motor_PWM);
}

// --------------------------------------------------------

//    ↑A-----B↓
//     |     |
//     |     |
//    ↑C-----D↓
void TURN_1()
{
  MOTORA_FORWARD(Motor_PWM);MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);MOTORD_BACKOFF(Motor_PWM);
}

// --------------------------------------------------------

//    ↓A-----B↑
//     |     |
//     |     |
//    ↓C-----D↑
void TURN_2()
{
  MOTORA_BACKOFF(Motor_PWM);MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);MOTORD_FORWARD(Motor_PWM);
}

// --------------------------------------------------------

void I2C_Control(char c)
{
  switch(c)
  {
     case 'A':  ADVANCE();  M_LOG("Run Forward!\r\n");    break;
     case 'B':  RIGHT_1();  M_LOG("Right up!\r\n");       break;
     case 'C':  RIGHT_2();  M_LOG("Right!\r\n");          break;
     case 'D':  RIGHT_3();  M_LOG("Right down!\r\n");     break;
     case 'E':  BACK();     M_LOG("Run Backwards!\r\n");  break;
     case 'F':  LEFT_1();   M_LOG("Left up!\r\n");        break;
     case 'G':  LEFT_2();   M_LOG("Left!\r\n");           break;
     case 'H':  LEFT_3();   M_LOG("Left down!\r\n");      break;
     case 'I':  TURN_1();   M_LOG("Turn Right!\r\n");     break;
     case 'J':  TURN_2();   M_LOG("Turn Left!\r\n");      break;
     case 'Z':  STOP();     M_LOG("Stop!\r\n");           break;
     case 'K':  Motor_PWM = 40;                           break;
     case 'L':  Motor_PWM = 240;                          break;
     case 'M':  Motor_PWM = 130;                          break;
   }
}

void IO_init()
{
	pinMode(PWMA, OUTPUT);
	pinMode(DIRA1, OUTPUT);pinMode(DIRA2, OUTPUT);
	pinMode(PWMB, OUTPUT);
	pinMode(DIRB1, OUTPUT);pinMode(DIRB2, OUTPUT);
  pinMode(PWMC, OUTPUT);
	pinMode(DIRC1, OUTPUT);pinMode(DIRC2, OUTPUT);
	pinMode(PWMD, OUTPUT);
	pinMode(DIRD1, OUTPUT);pinMode(DIRD2, OUTPUT);
	STOP();
}

void setup()
{
  Serial.begin(9600);
  IO_init();
  Wire.begin(0x18);
  Wire.onReceive(receiveEvent);
}

void receiveEvent(int numbBytes)
{
  received_msg = "";
  while(Wire.available() > 0){
    char c = Wire.read();
    if (c != '\n') received_msg.concat(c);
  }
  //Serial.println(received_msg);
  I2C_Control(received_msg[1]);
  Serial.println(received_msg[1]);
}

void loop()
{

}
