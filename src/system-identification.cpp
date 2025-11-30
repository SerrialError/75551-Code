#include "system-identification.hpp"

std::vector<double> SysIdent::calculate_mean_velocities(double voltage, std::vector<MotorController>& motors) {
	for (size_t i = 0; i < motors.size(); i++) {
		motors[i].move_motor_voltage(voltage);
	}
	pros::delay(1000);
	std::vector<double> total_velocities(motors.size(), 0.0);
	for (size_t i = 0; i < 50; i++) {
		for (size_t j = 0; j < motors.size(); j++) {
			double measured_rpm = motors[j].motor.get().get_actual_velocity();
			double measured_rad_s = measured_rpm * 2.0 * M_PI / 60.0;
			total_velocities[j] += measured_rad_s;
		}
		pros::delay(10);

	}
	std::vector<double> mean_velocities(motors.size(), 0.0);
	for (size_t i = 0; i < motors.size(); i++) {
		mean_velocities[i] = total_velocities[i] / 50.0;
	}
	return mean_velocities;
}

std::vector<std::pair<double,double>> SysIdent::calculate_Kv_and_Ks_s(std::vector<MotorController>& motors) {
	std::vector<std::vector<std::pair<double,double>>> velocity_data(motors.size());
	for (size_t i = 1; i <= 10; i++) {
		double applied_voltage = (i % 2 == 0) ? -static_cast<double>(i) : static_cast<double>(i);
		auto mean_ws = calculate_mean_velocities(applied_voltage, motors);
		for (size_t j = 0; j < motors.size(); j++) {
			velocity_data[j].push_back({ mean_ws[j], applied_voltage });
		}
	}
	for (size_t i = 0; i < motors.size(); i++) {
		motors[i].move_motor_voltage(0.0);
	}
	std::vector<std::pair<double, double>> Kv_and_Ks_s;
	Kv_and_Ks_s.reserve(motors.size());
	for (size_t i = 0; i < motors.size(); i++) {
		Kv_and_Ks_s.push_back(fit_Kv_and_Ks_with_sign(velocity_data[i]));
	}
	return Kv_and_Ks_s;
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
    if (std::abs(denom) < 1e-12) { // degenerate
	double b = (n > 0.0) ? (sum_y / n) : 0.0;
	return {0.0, b};
    }
    double m = (n * sum_xy - sum_x * sum_y) / denom;
	double b = (sum_y - m * sum_x) / n;

	return {m, b};
}

// Fit V = kV * w + kS * s  where s = sign(w)
std::pair<double,double> SysIdent::fit_Kv_and_Ks_with_sign(const std::vector<std::pair<double,double>>& points) {
    // points: (w, V)
    double sum_w = 0.0;
    double sum_s = 0.0;
    double sum_ww = 0.0;
    double sum_ws = 0.0;
    double sum_ss = 0.0;
    double sum_wy = 0.0;
    double sum_sy = 0.0;
    const double n = static_cast<double>(points.size());

    for (const auto &p : points) {
        double w = p.first;
        double y = p.second;
        double s = (w > 0.0) ? 1.0 : ((w < 0.0) ? -1.0 : 0.0);
        sum_w += w;
        sum_s += s;
        sum_ww += w * w;
        sum_ws += w * s;
        sum_ss += s * s; // equals count of nonzero sign entries
        sum_wy += w * y;
        sum_sy += s * y;
    }

    // Normal equations: [sum_ww sum_ws; sum_ws sum_ss] * [kV; kS] = [sum_wy; sum_sy]
    double det = sum_ww * sum_ss - sum_ws * sum_ws;
    if (std::abs(det) < 1e-12) {
        // degenerate: fall back to single-variable fit (your linear_reg) for kV, and approximate kS from medians
        auto kb = linear_reg(points); // {m,b} from V = m*w + b
        return { kb.first, kb.second }; // best-effort fallback
    }

    double kV =  ( sum_wy * sum_ss - sum_sy * sum_ws) / det;
    double kS =  (-sum_wy * sum_ws + sum_sy * sum_ww) / det;

    return {kV, kS};
}

std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>, std::vector<std::vector<double>>> SysIdent::get_acceleration_data(double voltage, std::vector<MotorController>& motors) {
	for (size_t i = 0; i < motors.size(); i++) {
		motors[i].move_motor_voltage(voltage);
	}
	std::vector<std::vector<double>> V(motors.size()), w(motors.size()), alpha(motors.size());
	std::vector<double> prev_w(motors.size(), 0.0);
	pros::delay(200);    
	bool have_prev = false;
	for (size_t i = 0; i < 300; i++) {
		for (size_t j = 0; j < motors.size(); j++) {
			double measured_mv = motors[j].motor.get().get_voltage();          // mV
			double measured_v = measured_mv / 1000.0;
			double measured_rpm = motors[j].motor.get().get_actual_velocity(); // RPM
			double measured_rad_s = measured_rpm * 2.0 * M_PI / 60.0;
			if (have_prev) {
				double measured_rad_s_s = (measured_rad_s - prev_w[j]) / 0.01;
				V[j].push_back(measured_v);
				w[j].push_back(measured_rad_s);
				alpha[j].push_back(measured_rad_s_s);
			}
			prev_w[j] = measured_rad_s;
		}
		have_prev = true;
		pros::delay(10);
	}
	return {V, w, alpha};
}

std::vector<double> SysIdent::calculate_Ka_s(std::vector<std::pair<double, double>> Kv_and_Ks_s, std::vector<MotorController>& motors) {
	auto [V, w, alpha] = get_acceleration_data(4.0, motors);
	std::vector<double> Ka;
	Ka.reserve(motors.size());
	for (size_t i = 0; i < motors.size(); i++) {
		Ka.push_back(through_origin_fit(V[i], w[i], alpha[i], Kv_and_Ks_s[i].first, Kv_and_Ks_s[i].second));
	}
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

std::vector<std::tuple<double, double, double>> SysIdent::calculate_Kv_Ka_and_Ks_s(std::vector<MotorController>& motors) {
	std::vector<std::pair<double, double>> Kv_and_Ks_s = calculate_Kv_and_Ks_s(motors);
	pros::delay(5000);
	std::vector<double> Ka = calculate_Ka_s(Kv_and_Ks_s, motors);
	std::vector<std::tuple<double, double, double>> Kv_Ka_and_Ks_s;
	for (size_t i = 0; i < motors.size(); i++) {
		Kv_Ka_and_Ks_s.emplace_back(Kv_and_Ks_s[i].first, Ka[i], Kv_and_Ks_s[i].second);
	}
	return Kv_Ka_and_Ks_s;
}

void SysIdent::calculate_and_print_constants(std::vector<MotorController>& motors) {
	auto results = calculate_Kv_Ka_and_Ks_s(motors);

	for (size_t i = 0; i < motors.size(); i++) {
		auto [Kv, Ka, Ks] = results[i];
		printf("%s\n", std::string(motors[i].motor_name).c_str());
		pros::delay(10);
		printf("K_v = ");
		pros::delay(10);
		printf("%.6f", Kv);
		pros::delay(10);
		printf("\nK_a = ");
		pros::delay(10);
		printf("%.6f", Ka);
		pros::delay(10);
		printf("\nK_s = ");
		pros::delay(10);
		printf("%.6f\n", Ks);
		pros::delay(10);
	}
}
