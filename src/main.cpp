#include <iostream>
#include "LQR.hpp"   // picks up DARE.hpp via include/DARE.hpp
#include <Eigen/Dense>

int main() {
    // example 2-state, 1-input LQR test
    Eigen::Matrix<double, 2, 2> A;
    Eigen::Matrix<double, 2, 1> B;
    Eigen::Matrix<double, 2, 2> Q;
    Eigen::Matrix<double, 1, 1> R;
    Eigen::Matrix<double, 2, 1> N;

    A << 1.0, 0.1,
         0.0, 1.0;
    B << 0.0,
         0.1;
    Q = Eigen::Matrix<double,2,2>::Identity();
    R << 1.0;
    N.setZero();

    auto K = LQR<2,1>(A, B, Q, R, N);

    std::cout << "K =\n" << K << std::endl;
    return 0;
}
