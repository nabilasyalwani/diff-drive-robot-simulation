#include <iostream>
#include <cmath>
#include <ctime>
#include <chrono>
#include <thread>

double dt = 0.1; // 0.1 detik

class PIDController
{
public:
    PIDController(double kp, double ki, double kd) : kp(kp), ki(ki), kd(kd), sum_error(0), prev_error(0) {}

    double calculate(double target, double posisi)
    {
        double error = target - posisi;
        sum_error += error;
        prev_error = error;

        double p = kp * error;
        double i = ki * sum_error;
        double d = kd * (error - prev_error) / dt;

        double output = p + i + d;

        if(abs(output) > 0.8) output = 0.8 * abs(output);

        std::cout << "p = " << p << std::endl;
        std::cout << "Error: " << error << std::endl;
        std::cout << "Sum Error: " << sum_error << std::endl;

        return output;
    }

private:
    double kp;
    double ki;
    double kd;
    double sum_error;
    double prev_error;
};

int main()
{
    double kp = 1.0;
    double ki = 0.1;
    double kd = 0.01;

    PIDController pid(kp, ki, kd);

    double target = 5.0;
    double posisi = 0.0;

    for (int i = 0; i < 20; ++i)
    {

        double control_output = pid.calculate(target, posisi);
        posisi += control_output;
        std::cout << " Control Output: " << control_output << std::endl;
    }

    return 0;
}
