/**
 * \file system-identification.cpp
 * @brief Implementation of motor system identification routines.
 *
 * Contains the procedures that sweep motor voltages, measure steady-state
 * speeds, and fit the feedforward model parameters (K_v, K_a, K_s) using
 * regression and filtered acceleration data.
 */
#include "system-identification.hpp"
#include <deque>

std::vector<float> SysIdent::calculate_mean_velocities(float voltage, std::vector<MotorController>& motors) {
	for (size_t i = 0; i < motors.size(); i++) {
		motors[i].move_voltage(voltage);
	}
	pros::delay(1000); // allow motors to reach steady state
	std::vector<float> total_velocities(motors.size(), 0.0);
	for (size_t i = 0; i < 50; i++) {
		for (size_t j = 0; j < motors.size(); j++) {
			float measured_rpm = motors[j].motor.get().get_actual_velocity();
			float measured_rad_s = measured_rpm * 2.0 * M_PI / 60.0;
			total_velocities[j] += measured_rad_s;
		}
		pros::delay(10);

	}
	std::vector<float> mean_velocities(motors.size(), 0.0);
	for (size_t i = 0; i < motors.size(); i++) {
		mean_velocities[i] = total_velocities[i] / 50.0;
	}
	return mean_velocities;
}

std::vector<std::pair<float,float>> SysIdent::calculate_Kv_and_Ks_s(std::vector<MotorController>& motors) {
	std::vector<std::vector<std::pair<float,float>>> velocity_data(motors.size());
	for (size_t i = 1; i <= 10; i++) {
		// Alternate between positive and negative voltages to identify K_s sign.
		float applied_voltage = (i % 2 == 0) ? -static_cast<float>(i) : static_cast<float>(i);
		auto mean_ws = calculate_mean_velocities(applied_voltage, motors);
		for (size_t j = 0; j < motors.size(); j++) {
			velocity_data[j].push_back({ mean_ws[j], applied_voltage });
			printf("STEADY motor=%s V=%.3f w=%.6f s=%d\n",
		  		std::string(motors[j].name).c_str(),
		  		applied_voltage,
		  		mean_ws[j],
		  		(mean_ws[j] > 0) - (mean_ws[j] < 0));
		}
	}
	for (size_t i = 0; i < motors.size(); i++) {
		motors[i].move_voltage(0.0);
	}
	std::vector<std::pair<float, float>> Kv_and_Ks_s;
	Kv_and_Ks_s.reserve(motors.size());
	for (size_t i = 0; i < motors.size(); i++) {
		Kv_and_Ks_s.push_back(fit_Kv_and_Ks_with_sign(velocity_data[i]));
	}
	return Kv_and_Ks_s;
}

std::pair<float,float> SysIdent::linear_reg(const std::vector<std::pair<float,float>>& points) {
	float sum_x = 0.0, sum_y = 0.0, sum_xx = 0.0, sum_xy = 0.0;
    const float n = static_cast<float>(points.size());

    for (const auto &point : points) {
        float x = point.first;
        float y = point.second;
        sum_x += x;
        sum_y += y;
        sum_xx += x * x;
        sum_xy += x * y;
    }

    float denom = n * sum_xx - sum_x * sum_x;
    if (std::abs(denom) < 1e-12) { // degenerate
	float b = (n > 0.0) ? (sum_y / n) : 0.0;
	return {0.0, b};
    }
    float m = (n * sum_xy - sum_x * sum_y) / denom;
	float b = (sum_y - m * sum_x) / n;

	return {m, b};
}

// Fit V = kV * w + kS * s  where s = sign(w)
std::pair<float,float> SysIdent::fit_Kv_and_Ks_with_sign(const std::vector<std::pair<float,float>>& points) {
    // points: (w, V)
    float sum_w = 0.0;
    float sum_s = 0.0;
    float sum_ww = 0.0;
    float sum_ws = 0.0;
    float sum_ss = 0.0;
    float sum_wy = 0.0;
    float sum_sy = 0.0;
    const float n = static_cast<float>(points.size());

    for (const auto &p : points) {
        float w = p.first;
        float y = p.second;
        float s = (w > 0.0) ? 1.0 : ((w < 0.0) ? -1.0 : 0.0);
        sum_w += w;
        sum_s += s;
        sum_ww += w * w;
        sum_ws += w * s;
        sum_ss += s * s; // equals count of nonzero sign entries
        sum_wy += w * y;
        sum_sy += s * y;
    }

    // Normal equations: [sum_ww sum_ws; sum_ws sum_ss] * [kV; kS] = [sum_wy; sum_sy]
    float det = sum_ww * sum_ss - sum_ws * sum_ws;
    if (std::abs(det) < 1e-12) {
        // degenerate: fall back to single-variable fit (your linear_reg) for kV, and approximate kS from medians
        auto kb = linear_reg(points); // {m,b} from V = m*w + b
        return { kb.first, kb.second }; // best-effort fallback
    }

    float kV =  ( sum_wy * sum_ss - sum_sy * sum_ws) / det;
    float kS =  (-sum_wy * sum_ws + sum_sy * sum_ww) / det;

    return {kV, kS};
}

std::tuple<std::vector<std::vector<float>>, std::vector<std::vector<float>>, std::vector<std::vector<float>>> SysIdent::get_acceleration_data(float voltage, std::vector<MotorController>& motors) {
    // Command motors
    for (size_t i = 0; i < motors.size(); i++) {
        motors[i].move_voltage(voltage);
    }

    const size_t iterations = 20;   // total outer loop iterations
    const int window = 4;            // N-sample window used for robust slope (3 is a good start)
    const float ms_to_s = 1.0 / 1000.0;

    // output buffers
    std::vector<std::vector<float>> V(motors.size()), w(motors.size()), alpha(motors.size());

    // small reserves to avoid realloc churn on embedded target
    for (size_t j = 0; j < motors.size(); ++j) {
        V[j].reserve(iterations / 2);
        w[j].reserve(iterations / 2);
        alpha[j].reserve(iterations / 2);
    }

    // rolling buffers for slope estimation
    std::vector<std::deque<float>> prev_ws(motors.size()), prev_ts(motors.size());

    // exponential filter state for w
    std::vector<float> w_filtered(motors.size(), 0.0);
    const float w_lp_alpha = 0.20; // filter coefficient (0..1). smaller => smoother but slower. tune if needed.

    // helper: current time in seconds
    auto now_seconds = []() -> float {
        return static_cast<float>(pros::millis()) * (1.0/1000.0);
    };

    // main sample loop
    for (size_t iter = 0; iter < iterations; ++iter) {
        float t = now_seconds();

        for (size_t j = 0; j < motors.size(); ++j) {
            float raw_voltage = motors[j].motor.get().get_voltage(); // could be mV or V
            // auto-detect units: if large (>100) assume mV, else assume volts
            float measured_v = (std::abs(raw_voltage) > 100.0) ? (raw_voltage / 1000.0) : raw_voltage;

            float measured_rpm = motors[j].motor.get().get_actual_velocity(); // RPM
            float measured_rad_s = measured_rpm * 2.0 * M_PI / 60.0;          // rad/s

            // exponential low-pass on w to reduce HF jitter
            // initialize filtered value on first useful sample
            if (iter == 0) w_filtered[j] = measured_rad_s;
            else w_filtered[j] = w_lp_alpha * measured_rad_s + (1.0 - w_lp_alpha) * w_filtered[j];

            // push filtered sample into rolling window for slope estimation
            prev_ws[j].push_back(w_filtered[j]);
            prev_ts[j].push_back(t);
            if (prev_ws[j].size() > static_cast<size_t>(window)) {
                prev_ws[j].pop_front();
                prev_ts[j].pop_front();
            }

            // when we have a full window, compute slope by small linear regression (least squares)
            if (prev_ws[j].size() == static_cast<size_t>(window)) {
                // compute means
                float mean_t = 0.0, mean_w = 0.0;
                for (int k = 0; k < window; ++k) {
                    mean_t += prev_ts[j][k];
                    mean_w += prev_ws[j][k];
                }
                mean_t /= static_cast<float>(window);
                mean_w /= static_cast<float>(window);

                float num = 0.0, den = 0.0;
                for (int k = 0; k < window; ++k) {
                    float dt = prev_ts[j][k] - mean_t;
                    float dw = prev_ws[j][k] - mean_w;
                    num += dt * dw;
                    den += dt * dt;
                }
                float measured_rad_s_s = (den != 0.0) ? (num / den) : 0.0; // rad/s^2

                // store the measurement triple
                V[j].push_back(measured_v);
                w[j].push_back(w_filtered[j]);           // push filtered w (not raw)
                alpha[j].push_back(measured_rad_s_s);
            }
            // otherwise we don't have enough samples yet to form a derivative point
        }

        pros::delay(10); // maintain ~10ms loop (actual dt comes from timestamps)
    }

    return {V, w, alpha};
}

std::vector<float> SysIdent::calculate_Ka_s(std::vector<std::pair<float, float>> Kv_and_Ks_s, std::vector<MotorController>& motors) {
    auto [V, w, alpha] = get_acceleration_data(10.0, motors);

    printf("---- BEGIN Ka DATA ----\n");

    std::vector<float> Ka;
    Ka.reserve(motors.size());

    // When calling through_origin_fit, use slightly stricter thresholds inside that function.
    // (If you haven't changed through_origin_fit, consider increasing alpha_thresh and w_thresh there.)
    for (size_t i = 0; i < motors.size(); i++) {
        // optional per-motor debug: print how many samples were captured for this motor
        printf("DBG Ka: motor=%s samples=%zu\n", std::string(motors[i].name).c_str(), alpha[i].size());
        Ka.push_back(through_origin_fit(V[i], w[i], alpha[i], Kv_and_Ks_s[i].first, Kv_and_Ks_s[i].second));
    }

    printf("---- END Ka DATA ----\n");
    return Ka;
}

float SysIdent::through_origin_fit(const std::vector<float>& V, const std::vector<float>& w, const std::vector<float>& alpha, float Kv, float Ks) {
	float alpha_thresh = 7.000;
	float w_thresh = 0.001;
	float num = 0.0, den = 0.0;

	for (size_t i = 0; i < V.size(); ++i) {
		// optionally ignore low-speed samples where sgn(w) is noisy:
		if (std::abs(w[i]) < w_thresh) continue;
		if (std::abs(alpha[i]) < alpha_thresh) continue; // no info about Ka

		float s = (w[i] > 0.0) ? 1.0 : -1.0; // choose sgn(0) handled by w_thresh
		float Y = V[i] - Kv * w[i] - Ks * s;
		num += alpha[i] * Y;
		den += alpha[i] * alpha[i];
		printf("ACCEL V=%.4f w=%.6f alpha=%.6f s=%d Y=%.6f\n",
		 V[i], w[i], alpha[i], (int)s, Y);
	}

	if (den == 0.0) return 0.0;

	float Ka = num / den;

	return Ka;
}

std::vector<std::tuple<float, float, float>> SysIdent::calculate_Kv_Ka_and_Ks_s(std::vector<MotorController>& motors) {
	std::vector<std::pair<float, float>> Kv_and_Ks_s = calculate_Kv_and_Ks_s(motors);
	pros::delay(1000);
	std::vector<float> Ka = calculate_Ka_s(Kv_and_Ks_s, motors);
	std::vector<std::tuple<float, float, float>> Kv_Ka_and_Ks_s;
	for (size_t i = 0; i < motors.size(); i++) {
		Kv_Ka_and_Ks_s.emplace_back(Kv_and_Ks_s[i].first, Ka[i], Kv_and_Ks_s[i].second);
	}
	return Kv_Ka_and_Ks_s;
}

void SysIdent::calculate_and_print_constants(std::vector<MotorController>& motors) {
	auto results = calculate_Kv_Ka_and_Ks_s(motors);

	for (size_t i = 0; i < motors.size(); i++) {
		auto [Kv, Ka, Ks] = results[i];
		printf("%s\n", std::string(motors[i].name).c_str());
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
