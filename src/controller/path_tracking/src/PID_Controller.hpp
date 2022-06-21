class PidController
{
    private:
        float kp;
        float ki;
        float kd;
        float Err;       //误差
        float ErrLastTime;  //上次的误差
        float ErrRate;  //误差变化
        int count;

    public:
        PidController();
        void Pid_Set(float p,float i,float d);
        float Pid_Control(float LosAngle,float theta);
};

PidController::PidController()
{
    kp = 0.035;
    ki = 0;
    kd = 0.023;
    Err = 0;
    ErrLastTime = 0;
    ErrRate = 0;
    count = 0;
}
float PidController::Pid_Control(float LosAngle,float theta)
{
    Err = (LosAngle - theta);
    if(Err< -180)
    {
        Err = Err + 360;
    }
    if(Err > 180)
    {
        Err = Err - 360;
    }

    if(count%10 == 0)
    {       
	    ErrRate = (Err - ErrLastTime)*5;
	    ErrLastTime = Err;
    }
    count++;
    float Angle =-(kp*Err+kd*ErrRate);
    return Angle;
}

void PidController::Pid_Set(float p,float i,float d)
{
    kp = p;
    ki = i;
    kd = d;
}