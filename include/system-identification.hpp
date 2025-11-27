#ifndef SYSTEM_IDENTIFICATION_HPP
#define SYSTEM_IDENTIFICATION_HPP

#include "api.h"
#include "structs.hpp"
#include "helper-functions.hpp"
#include "motor-controller.hpp"

class MotorController;

class SysIdent {
private:
	static std::pair<double,double> linear_reg(const std::vector<std::pair<double,double>>& points);
	static std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> get_acceleration_data(double voltage, MotorController& motor);
	static double through_origin_fit(const std::vector<double>& V, const std::vector<double>& w, const std::vector<double>& alpha, double Kv, double Ks);	
	static double calculate_mean_velocity(double voltage, MotorController& motor);
	static std::pair<double,double> calculate_Kv_and_Ks(MotorController& motor);
	static double calculate_Ka(std::pair<double,double> Kv_and_Ks, MotorController& motor);
	
public:
	static std::tuple<double, double, double> calculate_Kv_Ka_and_Ks(MotorController& motor);
};

#endif // SYSTEM_IDENTIFICATION_HPP
