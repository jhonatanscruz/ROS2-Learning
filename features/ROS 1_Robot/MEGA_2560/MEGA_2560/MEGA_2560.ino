#include <Encoder.h>

// Pinos do motor
#define PWMA 12  //A - Velocidade do motor
#define DIRA1 34
#define DIRA2 35 //A - Direção do motor
#define EN1_A 18
#define EN1_B 31

#define PWMB 8   //B - Velocidade do motor
#define DIRB1 37
#define DIRB2 36 //B - Direção do motor
#define EN2_A 19
#define EN2_B 38

#define PWMC 9   //C - Velocidade do motor
#define DIRC1 43
#define DIRC2 42 //C - Direção do motor
#define EN3_A 3
#define EN3_B 49

#define PWMD 5   //D - Velocidade do motor
#define DIRD1 A4
#define DIRD2 A5 //D - Direção do motor
#define EN4_A 2
#define EN4_B A1

#define MOTORA_FORWARD(pwm)    do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,HIGH); analogWrite(PWMA,pwm);}while(0)
#define MOTORA_STOP(x)         do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,LOW); analogWrite(PWMA,0);}while(0)
#define MOTORA_BACKOFF(pwm)    do{digitalWrite(DIRA1,HIGH);digitalWrite(DIRA2,LOW); analogWrite(PWMA,pwm);}while(0)

#define MOTORB_FORWARD(pwm)    do{digitalWrite(DIRB1,HIGH); digitalWrite(DIRB2,LOW); analogWrite(PWMB,pwm);}while(0)
#define MOTORB_STOP(x)         do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,LOW); analogWrite(PWMB,0);}while(0)
#define MOTORB_BACKOFF(pwm)    do{digitalWrite(DIRB1,LOW);digitalWrite(DIRB2,HIGH); analogWrite(PWMB,pwm);}while(0)

#define MOTORC_FORWARD(pwm)    do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,HIGH); analogWrite(PWMC,pwm);}while(0)
#define MOTORC_STOP(x)         do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,LOW); analogWrite(PWMC,0);}while(0)
#define MOTORC_BACKOFF(pwm)    do{digitalWrite(DIRC1,HIGH);digitalWrite(DIRC2,LOW); analogWrite(PWMC,pwm);}while(0)

#define MOTORD_FORWARD(pwm)    do{digitalWrite(DIRD1,HIGH); digitalWrite(DIRD2,LOW); analogWrite(PWMD,pwm);}while(0)
#define MOTORD_STOP(x)         do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,LOW); analogWrite(PWMD,0);}while(0)
#define MOTORD_BACKOFF(pwm)    do{digitalWrite(DIRD1,LOW);digitalWrite(DIRD2,HIGH); analogWrite(PWMD,pwm);}while(0)

#define SERIAL  Serial

#define LOG_DEBUG

#ifdef LOG_DEBUG
  #define M_LOG SERIAL.println
#else
  #define M_LOG 
#endif

#define MAX_PWM   200
#define MIN_PWM   130
int Motor_PWM = 60;

// ENCODERS
Encoder encoder_1(EN1_A, EN1_B);
Encoder encoder_2(EN2_A, EN2_B);
Encoder encoder_3(EN3_A, EN3_B);
Encoder encoder_4(EN4_A, EN4_B);
long oldPos = 0;

//controlar o movimento do motor - definição de macro 

//    ↑A-----B↑   
//     |  ↑  |
//     |  |  |
//    ↑C-----D↑
void ADVANCE()
{
  MOTORA_FORWARD(Motor_PWM);MOTORB_FORWARD(Motor_PWM);    
  MOTORC_FORWARD(Motor_PWM);MOTORD_FORWARD(Motor_PWM);
}

//    ↓A-----B↓   
//     |  |  |
//     |  ↓  |
//    ↓C-----D↓
void BACK()
{
  MOTORA_BACKOFF(Motor_PWM);MOTORB_BACKOFF(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);MOTORD_BACKOFF(Motor_PWM);
}
//    =A-----B↑   
//     |   ↖ |
//     | ↖   |
//    ↑C-----D=
void LEFT_1()
{
  MOTORA_STOP(Motor_PWM);MOTORB_FORWARD(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);MOTORD_STOP(Motor_PWM);
}

//    ↓A-----B↑   
//     |  ←  |
//     |  ←  |
//    ↑C-----D↓
void LEFT_2()
{
  MOTORA_BACKOFF(Motor_PWM);MOTORB_FORWARD(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);MOTORD_BACKOFF(Motor_PWM);
}
//    ↓A-----B=   
//     | ↙   |
//     |   ↙ |
//    =C-----D↓
void LEFT_3()
{
  MOTORA_BACKOFF(Motor_PWM);MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);MOTORD_BACKOFF(Motor_PWM);
}
//    ↑A-----B=   
//     | ↗   |
//     |   ↗ |
//    =C-----D↑
void RIGHT_1()
{
  MOTORA_FORWARD(Motor_PWM);MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);MOTORD_FORWARD(Motor_PWM);
}
//    ↑A-----B↓   
//     |  →  |
//     |  →  |
//    ↓C-----D↑
void RIGHT_2()
{
  MOTORA_FORWARD(Motor_PWM);MOTORB_BACKOFF(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);MOTORD_FORWARD(Motor_PWM);
}
//    =A-----B↓   
//     |   ↘ |
//     | ↘   |
//    ↓C-----D=
void RIGHT_3()
{
  MOTORA_STOP(Motor_PWM);MOTORB_BACKOFF(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);MOTORD_STOP(Motor_PWM);
}
// ↓A-----B↑
//  |     |
//  |     |
// ↓C-----D↑
void TURN_LEFT()
{
  MOTORA_BACKOFF(Motor_PWM);MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);MOTORD_FORWARD(Motor_PWM);  
}
// ↑A-----B↓
//  |     |
//  |     |
// ↑C-----D↓
void TURN_RIGHT()
{
  MOTORA_FORWARD(Motor_PWM);MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);MOTORD_BACKOFF(Motor_PWM);  
}
//    =A-----B=  
//     |  =  |
//     |  =  |
//    =C-----D=
void STOP()
{
  MOTORA_STOP(Motor_PWM);MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);MOTORD_STOP(Motor_PWM);
}

void UART_Control()
{
  if(SERIAL.available())
  {
    char Uart_Date = SERIAL.read();

    switch(Uart_Date)
    {
       case 'A':  ADVANCE(); M_LOG("Run!");         break;
       case 'B':  RIGHT_1();  M_LOG("Right up!");   break;
       case 'C':  RIGHT_2();  M_LOG("Right!");      break;
       case 'D':  RIGHT_3();  M_LOG("Right down!"); break;
       case 'E':  BACK();     M_LOG("Run!");        break;
       case 'F':  LEFT_3();   M_LOG("Left down!");  break;
       case 'G':  LEFT_2();   M_LOG("Left!");       break;
       case 'H':  LEFT_1();   M_LOG("Left up!");    break;
       case 'Z':  STOP();     M_LOG("Stop!");       break;
       case 'L':  Motor_PWM = 240;                  break;
       case 'M':  Motor_PWM = 130;                  break;
     }
  
    Serial.println(digitalRead(DIRA2));
    Serial.println(analogRead(PWMA));
  }
}

void IO_init()
{
  pinMode(PWMA, OUTPUT);
  pinMode(DIRA1, OUTPUT);
  pinMode(DIRA2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(DIRB1, OUTPUT);
  pinMode(DIRB2, OUTPUT);
  pinMode(PWMC, OUTPUT);
  pinMode(DIRC1, OUTPUT);
  pinMode(DIRC2, OUTPUT);
  pinMode(PWMD, OUTPUT);
  pinMode(DIRD1, OUTPUT);
  pinMode(DIRD2, OUTPUT);
  STOP();
}

void getEncoderPosition(Encoder myEnc, long oldPosition){
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(newPosition);
  }
}

void setup()
{
  Serial.begin(9600);
  while(!SERIAL);
  IO_init();
  Serial.println(digitalRead(DIRA2));
  Serial.println(analogRead(PWMA));
}

void loop() {

  //UART_Control();//Recepção e processamento de porta serial
  //ADVANCE();
  MOTORA_FORWARD(Motor_PWM);
  delay(5000);
  
  STOP();
  delay(1000);

  //BACK();
  MOTORB_FORWARD(Motor_PWM);
  delay(5000);
  
  STOP();
  delay(1000);

  //LEFT_2();
  MOTORC_FORWARD(Motor_PWM);
  delay(5000);
  
  STOP();
  delay(1000);

  //RIGHT_2();
  MOTORD_FORWARD(Motor_PWM);
  delay(5000);
  
  STOP();
  delay(1000);

  M_LOG(encoder.read());

  TURN_LEFT();
  delay(5000);

  STOP();
  delay(1000);

  M_LOG(encoder.read());

  TURN_RIGHT();
  delay(5000);

  STOP();
  delay(1000);

  M_LOG(encoder.read());
}
