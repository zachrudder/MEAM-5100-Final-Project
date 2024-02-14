class PIDController {
private:
    float Kp, Ki, Kd;
    float prevError, integral;
    float outputMin, outputMax;

public:
    // Constructor with output limits
    PIDController(float Kp, float Ki, float Kd, float outputMin, float outputMax)
        : Kp(Kp), Ki(Ki), Kd(Kd), prevError(0), integral(0), outputMin(outputMin), outputMax(outputMax) {}

    float compute(float setpoint, float measuredValue, float dt) {
        // Prevent division by zero
        if (dt <= 0) {
            return 0;
        }

        float error = setpoint - measuredValue;

        // Proportional term
        float Pout = Kp * error;

        // Integral term
        integral += error * dt;
        float Iout = Ki * integral;

        // Derivative term
        float derivative = (error - prevError) / dt;
        float Dout = Kd * derivative;

        prevError = error;

        // Constrain the integral term to prevent wind-up
        if (integral > outputMax) integral = outputMax;
        else if (integral < outputMin) integral = outputMin;

        // Calculate the PID output and constrain it within the specified limits
        float output = Pout + Iout + Dout;
        return constrain(output, outputMin, outputMax);
    }

    void reset() {
        prevError = 0;
        integral = 0;
    }
};
