#include "system-identification.hpp"

double SysIdent::calculate_mean_velocity(double voltage, MotorController& motor) {
	motor.move_motor_voltage(voltage);
	pros::delay(700);
	double mean_velocity = 0.0;
	for (size_t i = 0; i < 35; i++) {
    	double measured_rpm = motor.motor.get().get_actual_velocity(); // RPM
		double measured_rad_s = measured_rpm * 2.0 * M_PI / 60.0;
		mean_velocity += measured_rad_s;
		pros::delay(10);
	}
	mean_velocity = mean_velocity / 35.0;
	return mean_velocity;
}

std::pair<double,double> SysIdent::calculate_Kv_and_Ks(MotorController& motor) {
	std::vector<std::pair<double,double>> velocity_data;
	velocity_data.reserve(12);
	for (size_t i = 1; i <= 12; i++) {
		velocity_data.emplace_back(std::make_pair(calculate_mean_velocity(static_cast<double>(i), motor), static_cast<double>(i)));
	}
	motor.move_motor_voltage(0.0);
	std::pair<double,double> Kv_and_Ks = linear_reg(velocity_data);
	return Kv_and_Ks;
}

std::pair<double,double> SysIdent::linear_reg(const std::vector<std::pair<double,double>>& points) {
	double sum_x = 0.0, sum_y = 0.0, sum_xx = 0.0, sum_xy = 0.0;
    const double n = static_cast<double>(points.size());

    for (const auto &point : points) {
        double x = point.first;
        double y = point.second;
        sum_x += x;
        sum_y += y;
        sum_xx += x * x;
        sum_xy += x * y;
    }

    double denom = n * sum_xx - sum_x * sum_x;
    double m = (n * sum_xy - sum_x * sum_y) / denom;
	double b = (sum_y - m * sum_x) / n;

	return {m, b};
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> SysIdent::get_acceleration_data(double voltage, MotorController& motor) {
	motor.move_motor_voltage(voltage);
    std::vector<double> V;     V.reserve(40);
    std::vector<double> w;     w.reserve(40);
    std::vector<double> alpha; alpha.reserve(40);
	pros::delay(20);    
	double prev_w = 0.0;
    bool have_prev = false;
	for (size_t i = 0; i < 40; i++) {
    	double measured_mv = motor.motor.get().get_voltage();          // mV
		double measured_v = measured_mv / 1000.0;
    	double measured_rpm = motor.motor.get().get_actual_velocity(); // RPM
		double measured_rad_s = measured_rpm * 2.0 * M_PI / 60.0;
        if (have_prev) {
            double measured_rad_s_s = (measured_rad_s - prev_w) / 0.01;
			V.push_back(measured_v);
            w.push_back(measured_rad_s);
            alpha.push_back(measured_rad_s_s);
        }
        prev_w = measured_rad_s;
        have_prev = true;
		pros::delay(10);
	}
	return {V, w, alpha};
}

double SysIdent::calculate_Ka(std::pair<double,double> Kv_and_Ks, MotorController& motor) {
	auto [V, w, alpha] = get_acceleration_data(4.0, motor);
	double Ka = through_origin_fit(V, w, alpha, Kv_and_Ks.first, Kv_and_Ks.second);
	return Ka;
}

double SysIdent::through_origin_fit(const std::vector<double>& V, const std::vector<double>& w, const std::vector<double>& alpha, double Kv, double Ks) {
	double alpha_thresh = 0.001;
    double w_thresh = 0.001;
    double num = 0.0, den = 0.0;

    for (size_t i = 0; i < V.size(); ++i) {
        // optionally ignore low-speed samples where sgn(w) is noisy:
        if (std::abs(w[i]) < w_thresh) continue;
        if (std::abs(alpha[i]) < alpha_thresh) continue; // no info about Ka

        double s = (w[i] > 0.0) ? 1.0 : -1.0; // choose sgn(0) handled by w_thresh
        double Y = V[i] - Kv * w[i] - Ks * s;
        num += alpha[i] * Y;
        den += alpha[i] * alpha[i];
    }

    if (den == 0.0) return 0.0;

    double Ka = num / den;

    return Ka;
}

std::tuple<double, double, double> SysIdent::calculate_Kv_Ka_and_Ks(MotorController& motor) {
	std::pair<double, double> Kv_and_Ks = calculate_Kv_and_Ks(motor);
	double Ka = calculate_Ka(Kv_and_Ks, motor);
	std::tuple<double, double, double> Kv_Ka_and_Ks = {Kv_and_Ks.first, Ka, Kv_and_Ks.second};
	return Kv_Ka_and_Ks;
}
