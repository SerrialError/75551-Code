#include "drivetrain.hpp"
#include "simplex.hpp"

wheel_vels drivetrain::calculate_wheel_vels(pose_vels desired_vels,
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
    double t = std::tan(desired_vels.theta_vel);
    std::vector<double> a_ang = {1+t, 1-t, 1-t, 1+t, 1, 1};
    A.push_back(a_ang); b.push_back(0.0);
    for (double &v : a_ang) v = -v;
    A.push_back(a_ang); b.push_back(0.0);

    // Torque constraint
    double L = wheelbase_length;
    double W = trackwidth_length;
    std::vector<double> a_tau = {-(L+W)/4, (L+W)/4, -(L+W)/4, (L+W)/4, -W/2, W/2};
    A.push_back(a_tau); b.push_back(desired_vels.theta_vel);
    for (double &v : a_tau) v = -v;
    A.push_back(a_tau); b.push_back(desired_vels.theta_vel);

    // Solve LP
    std::vector<double> sol = Simplex::solve(A, b, c);

    // Map solution into wheel_vels
    wheel_vels result;
    result.m1_vel = sol[0];
    result.m2_vel = sol[1];
    result.m3_vel = sol[2];
    result.m4_vel = sol[3];
    result.o1_vel = sol[4];
    result.o2_vel = sol[5];
    return result;
}
