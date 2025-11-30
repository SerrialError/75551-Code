#ifndef SYSTEM_IDENTIFICATION_HPP
#define SYSTEM_IDENTIFICATION_HPP

#include "api.h"
#include "structs.hpp"
#include "helper-functions.hpp"
#include "motor-controller.hpp"

class MotorController;

class SysIdent {
private:
	static std::vector<double> calculate_mean_velocities(double voltage, std::vector<MotorController>& motors);

	static std::vector<std::pair<double,double>> calculate_Kv_and_Ks_s(std::vector<MotorController>& motors);

	static std::pair<double,double> linear_reg(const std::vector<std::pair<double,double>>& points);

	static std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>, std::vector<std::vector<double>>> get_acceleration_data(double voltage, std::vector<MotorController>& motors);

	static std::vector<double> calculate_Ka_s(std::vector<std::pair<double, double>> Kv_and_Ks_s, std::vector<MotorController>& motors);

	static double through_origin_fit(const std::vector<double>& V, const std::vector<double>& w, const std::vector<double>& alpha, double Kv, double Ks);

public:
	static std::vector<std::tuple<double, double, double>> calculate_Kv_Ka_and_Ks_s(std::vector<MotorController>& motors);
	static void calculate_and_print_constants(std::vector<MotorController>& motors);
};

#endif // SYSTEM_IDENTIFICATION_HPP
