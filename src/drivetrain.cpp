#include "drivetrain.hpp"
#include "simplex.hpp"

wheels drivetrain::calculate_wheel_vels(pose desired_vels,
                                            wheel_vel_lims limits) {
    // Decision vars: [m1, m2, m3, m4, o1, o2]
    const int n = 6;

    // Objective c^T x = 2*m2 + 2*m3 + o1 + o2
    std::vector<double> c = {0, 2, 2, 0, 1, 1};

    // Constraint matrix A and vector b
    std::vector<std::vector<double>> A;
    std::vector<double> b;

    // Add bounds: F_i <= max, -F_i <= -min
    std::vector<wheel_vel_lim> lims = {
        limits.m1_limits, limits.m2_limits, limits.m3_limits,
        limits.m4_limits, limits.o1_limits, limits.o2_limits
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
    wheels result;
    result.m1 = sol[0];
    result.m2 = sol[1];
    result.m3 = sol[2];
    result.m4 = sol[3];
    result.o1 = sol[4];
    result.o2 = sol[5];
    return result;
}

void drivetrain::move_wheel_accels(wheels wheel_accelerations) {
    double m1_velocity = m1.get_actual_velocity() * 2.f * M_PI / 60.f;
    double m1_voltage = m1_ff.compute_voltage(wheel_accelerations.m1, m1_velocity);
    double m2_velocity = m2.get_actual_velocity() * 2.f * M_PI / 60.f;
    double m2_voltage = m2_ff.compute_voltage(wheel_accelerations.m2, m2_velocity);
    double o1_velocity = o1.get_actual_velocity() * 2.f * M_PI / 60.f;
    double o1_voltage = o1_ff.compute_voltage(wheel_accelerations.o1, o1_velocity);
    double o2_velocity = o2.get_actual_velocity() * 2.f * M_PI / 60.f;
    double o2_voltage = o2_ff.compute_voltage(wheel_accelerations.m1, m2_velocity);
    double m3_velocity = m3.get_actual_velocity() * 2.f * M_PI / 60.f;
    double m3_voltage = m3_ff.compute_voltage(wheel_accelerations.m3, m3_velocity);
    double m4_velocity = m4.get_actual_velocity() * 2.f * M_PI / 60.f;
    double m4_voltage = m4_ff.compute_voltage(wheel_accelerations.m4, m4_velocity);
    m1.move_voltage(m1_voltage);
    m2.move_voltage(m2_voltage);
    o1.move_voltage(o1_voltage);
    o2.move_voltage(o2_voltage);
    m3.move_voltage(m3_voltage);
    m4.move_voltage(m4_voltage);
}

void drivetrain::field_oriented_holonomic_control(double x_1_axis, double y_1_axis, double x_2_axis, double y_2_axis) {

}
