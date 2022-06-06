#include "onboard_MAT01.h"

typedef DC_Motor Motor_Class;

class Mecanum_Car
{
public:
    int Now_spd;
    Mecanum_Car(Motor_Class *_LF_Wheel, Motor_Class *_RF_Wheel, Motor_Class *_LR_Wheel, Motor_Class *_RR_Wheel)
    {
        this->LF_Wheel = _LF_Wheel;
        this->RF_Wheel = _RF_Wheel;
        this->LR_Wheel = _LR_Wheel;
        this->RR_Wheel = _RR_Wheel;
    }

    void Init(void)
    {
        LF_Wheel->Init(0); 
        RF_Wheel->Init(1); 
        LR_Wheel->Init(0); 
        RR_Wheel->Init(1); //初始化电机PWM
    }

    void SetSpd(int Spd)
    {
        LF_Wheel_Spd = RF_Wheel_Spd = LR_Wheel_Spd = RR_Wheel_Spd = constrain(Spd, -MAX_RPM, MAX_RPM);  
    }

    void ROS_MoveBase(float Line_vel, float Pan_vel, float Angle_vel)
    {
        Line_vel *= cos(45);
        Pan_vel *= sin(45);
        LF_Wheel_Spd = constrain((Line_vel - Pan_vel + Angle_vel * 0.25) * 30, -MAX_RPM, MAX_RPM);
        RF_Wheel_Spd = constrain((Line_vel + Pan_vel + Angle_vel * 0.25) * 30, -MAX_RPM, MAX_RPM);
        LR_Wheel_Spd = constrain((Line_vel - Pan_vel - Angle_vel * 0.25) * 30, -MAX_RPM, MAX_RPM);
        RR_Wheel_Spd = constrain((Line_vel + Pan_vel - Angle_vel * 0.25) * 30, -MAX_RPM, MAX_RPM);
    }

    void ClearOdom()
    {
         LF_Wheel->clrEncoderPosition();
         RF_Wheel->clrEncoderPosition();
         LR_Wheel->clrEncoderPosition();
         RR_Wheel->clrEncoderPosition();
    }

    void ReadOdom()
    {
        Serial.print(RR_Wheel->getEncoderPosition());
        Serial.print(", ");
        Serial.print(RR_Wheel->getMotorRPM());
        Serial.println(", ");
    }

    void Increment_PID(void)
    {
        LF_Wheel->Incremental_PID(LF_Wheel_Spd);
        RF_Wheel->Incremental_PID(RF_Wheel_Spd);
        LR_Wheel->Incremental_PID(LR_Wheel_Spd);
        RR_Wheel->Incremental_PID(RR_Wheel_Spd);
    }

    void Update_PID(float _kp, float _ki, float _kd)
    {
          LF_Wheel->Update_PID(_kp, _ki, _kd);
          RF_Wheel->Update_PID(_kp, _ki, _kd);
          LR_Wheel->Update_PID(_kp, _ki, _kd);
          RR_Wheel->Update_PID(_kp, _ki, _kd);
    }

private:
    Motor_Class *LF_Wheel;
    Motor_Class *RF_Wheel;
    Motor_Class *LR_Wheel;
    Motor_Class *RR_Wheel;
    
    int LR_Wheel_Spd;
    int RR_Wheel_Spd;
    int RF_Wheel_Spd;
    int LF_Wheel_Spd;
};
