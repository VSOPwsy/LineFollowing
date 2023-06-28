struct PID {
    float desired_angle;  // Desired angle
    float desired_distance; // Desired distance from center
    float previous_angle_error; // Angle error from previous cycle
    float previous_distance_error; // Distance error from previous cycle
    float integral_angle; // Sum of angle errors
    float integral_distance; // Sum of distance errors
    float Kp_angle, Ki_angle, Kd_angle; // PID coefficients for angle
    float Kp_distance, Ki_distance, Kd_distance; // PID coefficients for distance
};

struct PID pid_control;

void Line_Following_init(void);

void update_pid(struct PID *pid, float current_angle, float current_distance, void (*motor_function)(int, int));
void Motor_Rotation(int angle, int distance);
void Motor_Left_Front(int speed);
void Motor_Right_Front(int speed);
void Motor_Left_Rear(int speed);
void Motor_Right_Rear(int speed);
