#include "drivetrain.hpp"
#include "simplex.hpp"

wheels<double> drivetrain::calculate_wheel_vels(const pose& desired_vels, const wheels<wheel_vel_lim>& limits) {
    // Decision vars: [m1, m2, m3, m4, o1, o2]
    const int n = 6;

    // Objective c^T x = 2*m2 + 2*m3 + o1 + o2
    std::vector<double> c = {0, 2, 2, 0, 1, 1};

    // Constraint matrix A and vector b
    std::vector<std::vector<double>> A;
    std::vector<double> b;

    // Add bounds: F_i <= max, -F_i <= -min
    std::vector<wheel_vel_lim> lims = {
        limits.m1, limits.m2, limits.m3,
        limits.m4, limits.o1, limits.o2
    };

    for (int i = 0; i < n; i++) {
        std::vector<double> row(n, 0.0);
        row[i] = 1.0; A.push_back(row); b.push_back(lims[i].max);
        row[i] = -1.0; A.push_back(row); b.push_back(-lims[i].min);
    }

    // Angle constraint (linearized)
    double t = std::tan(desired_vels.theta);
    std::vector<double> a_ang = {1+t, 1-t, 1-t, 1+t, 1, 1};
    A.push_back(a_ang); b.push_back(0.0);
    for (double &v : a_ang) v = -v;
    A.push_back(a_ang); b.push_back(0.0);

    // Torque constraint
    double L = wheelbase_length;
    double W = trackwidth_length;
    std::vector<double> a_tau = {-(L+W)/4, (L+W)/4, -(L+W)/4, (L+W)/4, -W/2, W/2};
    A.push_back(a_tau); b.push_back(desired_vels.theta);
    for (double &v : a_tau) v = -v;
    A.push_back(a_tau); b.push_back(desired_vels.theta);

    // Solve LP
    std::vector<double> sol = Simplex::solve(A, b, c);

    // Map solution into wheels vels
    wheels<double> result;
    result.m1 = sol[0];
    result.m2 = sol[1];
    result.m3 = sol[2];
    result.m4 = sol[3];
    result.o1 = sol[4];
    result.o2 = sol[5];
    return result;
}

void drivetrain::move_wheel_accels(const wheels<double>& wheel_accelerations) {
    double m1_velocity = motors.m1.get().get_actual_velocity() * 2.f * M_PI / 60.f;
    double m1_voltage = motor_ffs.m1.compute_voltage(wheel_accelerations.m1, m1_velocity) * 1000.f;
    double m2_velocity = motors.m2.get().get_actual_velocity() * 2.f * M_PI / 60.f;
    double m2_voltage = motor_ffs.m2.compute_voltage(wheel_accelerations.m2, m2_velocity) * 1000.f;
    double o1_velocity = motors.o1.get().get_actual_velocity() * 2.f * M_PI / 60.f;
    double o1_voltage = motor_ffs.o1.compute_voltage(wheel_accelerations.o1, o1_velocity) * 1000.f;
    double o2_velocity = motors.o2.get().get_actual_velocity() * 2.f * M_PI / 60.f;
    double o2_voltage = motor_ffs.o2.compute_voltage(wheel_accelerations.m1, m2_velocity) * 1000.f;
    double m3_velocity = motors.m3.get().get_actual_velocity() * 2.f * M_PI / 60.f;
    double m3_voltage = motor_ffs.m3.compute_voltage(wheel_accelerations.m3, m3_velocity) * 1000.f;
    double m4_velocity = motors.m4.get().get_actual_velocity() * 2.f * M_PI / 60.f;
    double m4_voltage = motor_ffs.m4.compute_voltage(wheel_accelerations.m4, m4_velocity) * 1000.f;
    motors.m1.get().move_voltage(static_cast<int>(std::lround(m1_voltage)));
    motors.m2.get().move_voltage(static_cast<int>(std::lround(m2_voltage)));
    motors.o1.get().move_voltage(static_cast<int>(std::lround(o1_voltage)));
    motors.o2.get().move_voltage(static_cast<int>(std::lround(o2_voltage)));
    motors.m3.get().move_voltage(static_cast<int>(std::lround(m3_voltage)));
    motors.m4.get().move_voltage(static_cast<int>(std::lround(m4_voltage)));
}

void drivetrain::field_oriented_holonomic_control(void) {

}
