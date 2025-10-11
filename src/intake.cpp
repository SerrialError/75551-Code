#include "intake.hpp"

void intake::move_wheel_accels(const rollers<double>& wheel_accelerations) {
    double fb_velocity = motors.fb.get().get_actual_velocity() * 2.f * M_PI / 60.f;
    double fb_voltage = motor_ffs.fb.compute_voltage(wheel_accelerations.fb, fb_velocity) * 1000.f;
    double ft_velocity = motors.ft.get().get_actual_velocity() * 2.f * M_PI / 60.f;
    double ft_voltage = motor_ffs.ft.compute_voltage(wheel_accelerations.ft, ft_velocity) * 1000.f;
    double bb_velocity = motors.bb.get().get_actual_velocity() * 2.f * M_PI / 60.f;
    double bb_voltage = motor_ffs.bb.compute_voltage(wheel_accelerations.bb, bb_velocity) * 1000.f;
    double bt_velocity = motors.bt.get().get_actual_velocity() * 2.f * M_PI / 60.f;
    double bt_voltage = motor_ffs.bt.compute_voltage(wheel_accelerations.bt, bt_velocity) * 1000.f;
    motors.fb.get().move_voltage(static_cast<int>(std::lround(fb_voltage)));
    motors.ft.get().move_voltage(static_cast<int>(std::lround(ft_voltage)));
    motors.bb.get().move_voltage(static_cast<int>(std::lround(bb_voltage)));
    motors.bt.get().move_voltage(static_cast<int>(std::lround(bt_voltage)));
}

void intake::move_wheel_volts(const rollers<double>& wheel_voltages) {
    motors.fb.get().move_voltage(static_cast<int>(std::lround(wheel_voltages.fb)));
    motors.ft.get().move_voltage(static_cast<int>(std::lround(wheel_voltages.ft)));
    motors.bb.get().move_voltage(static_cast<int>(std::lround(wheel_voltages.bb)));
    motors.bt.get().move_voltage(static_cast<int>(std::lround(wheel_voltages.bt)));
}

rollers<wheel_vel_lim> intake::get_motor_vel_limits(const double& dt) {
    rollers<wheel_vel_lim> result{};
    result.fb.min = motors.fb.get().get_actual_velocity() * 2.f * M_PI / 60.f - motor_constants.fb.max_ang_accel * dt;
    result.ft.min = motors.ft.get().get_actual_velocity() * 2.f * M_PI / 60.f - motor_constants.ft.max_ang_accel * dt;
    result.bb.min = motors.bb.get().get_actual_velocity() * 2.f * M_PI / 60.f - motor_constants.bb.max_ang_accel * dt;
    result.bt.min = motors.bt.get().get_actual_velocity() * 2.f * M_PI / 60.f - motor_constants.bt.max_ang_accel * dt;
    result.fb.max = motors.fb.get().get_actual_velocity() * 2.f * M_PI / 60.f + motor_constants.fb.max_ang_accel * dt;
    result.ft.max = motors.ft.get().get_actual_velocity() * 2.f * M_PI / 60.f + motor_constants.ft.max_ang_accel * dt;
    result.bb.max = motors.bb.get().get_actual_velocity() * 2.f * M_PI / 60.f + motor_constants.bb.max_ang_accel * dt;
    result.bt.max = motors.bt.get().get_actual_velocity() * 2.f * M_PI / 60.f + motor_constants.bt.max_ang_accel * dt;
    return result;
}

rollers<double> intake::get_wanted_motor_accels(const rollers<double>& wanted_motor_vels, const double& dt) {
    rollers<double> result{};
    result.fb = (motors.fb.get().get_actual_velocity() * 2.f * M_PI / 60.f - wanted_motor_vels.fb) / dt;
    result.ft = (motors.ft.get().get_actual_velocity() * 2.f * M_PI / 60.f - wanted_motor_vels.ft) / dt;
    result.bb = (motors.bb.get().get_actual_velocity() * 2.f * M_PI / 60.f - wanted_motor_vels.bb) / dt;
    result.bt = (motors.bt.get().get_actual_velocity() * 2.f * M_PI / 60.f - wanted_motor_vels.bt) / dt;
    return result;
}

rollers<motorStateType> intake::get_roller_states(void) {
    rollers<motorStateType> result;
    switch (intakeState) {
        case off:
            result = {false, false, false, false};
            break;
        case intakeOnly:
            result = {'R', true, 'R', true};
            break;
    
        default:
            result = {0, 0, 0, 0};
            break;
    }
    return result;
}