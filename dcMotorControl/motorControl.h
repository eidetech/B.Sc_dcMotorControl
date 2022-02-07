// Left encoder pins
#define ENCA_L 19
#define ENCB_L 18

// Right encoder pins
#define ENCA_R 3
#define ENCB_R 2

// Left motor pins
#define PWM_L 7
#define IN2_L 8
#define IN1_L 9

// Right motor pins
#define PWM_R 4
#define IN2_R 5
#define IN1_R 6

enum motor {LEFT, RIGHT};

class MotorControl
{
private:
    
public:
    MotorControl();
    ~MotorControl();

    float pid(float setpointL, float setpointR, motor motorSide);
    static void encoder_ISR_L();
    static void encoder_ISR_R();
    void runMotor(int dir, int pwmVal, int pwmPin, int in1, int in2);

    // PID parameters
    float kp_L = 4;
    float ki_L = 0;
    float kd_L = 0;

    float kp_R = 3;
    float ki_R = 0;
    float kd_R = 0;

    long t_prev = 0;
    float eprev = 0;
    float eintegral = 0;
};
