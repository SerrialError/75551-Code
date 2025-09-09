#include "system-identification.hpp"

std::vector<input_output> compute::fopdt_system_identification(int n) {
    std::vector<input_output> result;
    for (int i = -12; i < 13; i++) {
        test_motor.move_voltage(1000*i);
        for (int j = 0; j < n; j++) {            
            double measured_mv = test_motor.get_voltage();           // mV
                    double measured_rpm = test_motor.get_actual_velocity(); // RPM
                    input_output sample;
                    sample.u = measured_mv / 1000.f;             // V
                    sample.x = measured_rpm * 2.f * M_PI / 60.f; // rad/s
                    result.push_back(sample);
            pros::delay(10);
        };
    };
    test_motor.move_voltage(0);
    return result;  
};


void print_vector(const std::vector<input_output>& vec) {
    printf("U = [");
    for (size_t i = 0; i < vec.size(); ++i) {
        printf("%.6f", vec[i].u);
        if (i != vec.size() - 1) printf(",");
    }
    printf("]\n");
    printf("X = [");
    for (size_t i = 0; i < vec.size(); ++i) {
        printf("%.6f", vec[i].x);
        if (i != vec.size() - 1) printf(",");
    }
    printf("]\n");
}
