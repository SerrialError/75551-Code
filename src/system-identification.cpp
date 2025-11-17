#include "system-identification.hpp"

void print_data::print_vector() {
    printf("%s\n", motor.motor_name);
    printf("U = [");
    for (size_t i = 0; i < motor_data.size(); ++i) {
        printf("%.6f", motor_data[i].u);
        if (i != motor_data.size() - 1) printf(",");
    }
    printf("]\n");
    printf("X = [");
    for (size_t i = 0; i < motor_data.size(); ++i) {
        printf("%.6f", motor_data[i].x);
        if (i != motor_data.size() - 1) printf(",");
    }
    printf("]\n");
}

void print_data::update_motor_data() {
    double measured_mv = motor.motor.get().get_voltage();           // mV
    double measured_rpm = motor.motor.get().get_actual_velocity(); // RPM
    input_output sample;
    sample.u = measured_mv / 1000.f;             // V
    sample.x = measured_rpm * 2.f * M_PI / 60.f; // rad/s
    motor_data.push_back(sample);
};