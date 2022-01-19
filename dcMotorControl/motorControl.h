class MotorControl
{
private:
    
public:
    MotorControl();
    ~MotorControl();

    void pid(float setpoint);

    // PID parameters
    float kp = 1;
    float ki = 0.05;
    float kd = 0.06;

    long t_prev = 0;
    float eprev = 0;
    float eintegral = 0;
};