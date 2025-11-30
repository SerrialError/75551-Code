#include "system-identification.hpp"

std::vector<double> SysIdent::calculate_mean_velocities(double voltage, std::vector<MotorController>& motors) {
	for (size_t i = 0; i < motors.size(); i++) {
	    motors[i].move_motor_voltage(voltage);
	}
    	pros::delay(50);
	std::vector<double> total_velocities(motors.size(), 0.0);
	for (size_t i = 0; i < 35; i++) {
		for (size_t j = 0; j < motors.size(); j++) {
    			double measured_rpm = motors[j].motor.get().get_actual_velocity();
			double measured_rad_s = measured_rpm * 2.0 * M_PI / 60.0;
			total_velocities[j] += measured_rad_s;
		}
    		pros::delay(10);

	}
	std::vector<double> mean_velocities(motors.size(), 0.0);
	for (size_t i = 0; i < motors.size(); i++) {
		mean_velocities[i] = total_velocities[i] / 35.0;
	}
	return mean_velocities;
}

std::vector<std::pair<double,double>> SysIdent::calculate_Kv_and_Ks_s(std::vector<MotorController>& motors) {
	std::vector<std::vector<std::pair<double,double>>> velocity_data(motors.size());
	for (size_t i = 1; i <= 12; i++) {
		auto mean_ws = calculate_mean_velocities(static_cast<double>(i), motors);
    		for (size_t j = 0; j < motors.size(); j++) {
        		velocity_data[j].push_back({ mean_ws[j], static_cast<double>(i) });
   		}
	}
	for (size_t i = 0; i < motors.size(); i++) {
	    motors[i].move_motor_voltage(0.0);
	}
	std::vector<std::pair<double, double>> Kv_and_Ks_s;
	Kv_and_Ks_s.reserve(motors.size());
	for (size_t i = 0; i < motors.size(); i++) {
	    Kv_and_Ks_s.push_back(linear_reg(velocity_data[i]));
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

std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>, std::vector<std::vector<double>>> SysIdent::get_acceleration_data(double voltage, std::vector<MotorController>& motors) {
	for (size_t i = 0; i < motors.size(); i++) {
	    motors[i].move_motor_voltage(voltage);
	}
	std::vector<std::vector<double>> V(motors.size()), w(motors.size()), alpha(motors.size());
	std::vector<double> prev_w(motors.size(), 0.0);
	pros::delay(200);    
    	bool have_prev = false;
	for (size_t i = 0; i < 40; i++) {
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
