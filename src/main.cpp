#include "main.h"
pros::adi::Pneumatics redirector ('b', false);
pros::adi::Pneumatics descorer ('b', true);
pros::Rotation linear_wheel(15);
pros::Rotation horizontal_wheel(5);
pros::Imu imu_sensor_1(3);
pros::Imu imu_sensor_2(6);

pros::Motor m1(-13, pros::v5::MotorGears::blue);
pros::Motor m2(18, pros::v5::MotorGears::blue);
pros::Motor o1(-12, pros::v5::MotorGears::blue);
pros::Motor o2(19, pros::v5::MotorGears::blue);
pros::Motor m3(-11, pros::v5::MotorGears::blue);
pros::Motor m4(20, pros::v5::MotorGears::blue);

const wheels<std::reference_wrapper<pros::Motor>> driveMotors{
	std::ref(m1),
	std::ref(m2),
	std::ref(o1),
	std::ref(o2),
	std::ref(m3),
	std::ref(m4)
};

// kA, kV, kS
const wheels<ff_constants> dtConsts{
	{0.0045870351481, 0.166157827462, 0.120535260966, 75.858991, 12.24},
	{0.00346235990182, 0.16618573907, 0.103286863565, 74.351026, 12.055},
	{0.0011750526572, 0.156487380984, 0.133576249675, 80.215332, 12.055},
	{0.00169461151421, 0.161775364104, 0.136640169512, 76.235982, 12.055},
	{0.0043200416094, 0.16243797244, 0.259385827274, 75.314448, 12.012},
	{0.00101968096594, 0.157778384448, 0.16310482292, 78.351321, 12.012}
};


const double wheelbase = 0.292100005; // m
const double trackwidth = 0.29508135; // m

drivetrain dt(driveMotors, wheelbase, trackwidth, dtConsts, linear_wheel, horizontal_wheel, imu_sensor_1, {0, 0, 0});

pros::Motor fb(8, pros::v5::MotorGears::blue);
pros::Motor ft(2, pros::v5::MotorGears::blue);
pros::Motor bb(1, pros::v5::MotorGears::blue);
pros::Motor bt(10, pros::v5::MotorGears::blue);

const rollers<std::reference_wrapper<pros::Motor>> intakeMotors{
	std::ref(fb),
	std::ref(ft),
	std::ref(bb),
	std::ref(bt)
};
pros::Optical optical(4);
const rollers<ff_constants> intakeConsts {
	{0.00681526983289, 0.276443936704, 0.23522177916, 27.590165, 7.618},
	{0.00681526983289, 0.276443936704, 0.23522177916, 27.590165, 7.618},   	 
	{0.00291077896309, 0.26137479893, 0.0463373039661, 31.360076, 7.585},
	{0.00744674908847, 0.265135278752, 0.17548243974, 27.646015, 7.294}
};

intake Intake(intakeMotors, optical, intakeConsts);

const double dt_ = 0.01;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
        pros::lcd::set_text(2, "I was pressed!");
    } else {
        pros::lcd::clear_line(2);
    }
}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello PROS User!");


    pros::lcd::register_btn1_cb(on_center_button);
}


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}


/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}


/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	std::vector<pose> P = {{1.426000, -0.754000, 2.922142},{1.424524, -0.753671, 2.922142},{1.421572, -0.753012, 2.922142},{1.417144, -0.752025, 2.922142},{1.411241, -0.750708, 2.922142},{1.403861, -0.749062, 2.922142},{1.395005, -0.747087, 2.922142},{1.384674, -0.744783, 2.922142},{1.372868, -0.742149, 2.922142},{1.359584, -0.739186, 2.922141},{1.344825, -0.735894, 2.922141},{1.328590, -0.732273, 2.922141},{1.310878, -0.728323, 2.922141},{1.291691, -0.724043, 2.922140},{1.271028, -0.719435, 2.922139},{1.248889, -0.714496, 2.922138},{1.225274, -0.709229, 2.922137},{1.200184, -0.703633, 2.922135},{1.175092, -0.698036, 2.922133},{1.150001, -0.692440, 2.922131},{1.124911, -0.686843, 2.922128},{1.099820, -0.681246, 2.922125},{1.074729, -0.675649, 2.922121},{1.049639, -0.670053, 2.922116},{1.024548, -0.664455, 2.922111},{0.999457, -0.658858, 2.922104},{0.974367, -0.653261, 2.922097},{0.949276, -0.647663, 2.922088},{0.924185, -0.642066, 2.922079},{0.899095, -0.636467, 2.922068},{0.874004, -0.630869, 2.922056},{0.848915, -0.625271, 2.922044},{0.823824, -0.619672, 2.922030},{0.798734, -0.614072, 2.922015},{0.773644, -0.608473, 2.922000},{0.748554, -0.602873, 2.921984},{0.723464, -0.597272, 2.921969},{0.698374, -0.591671, 2.921954},{0.673284, -0.586070, 2.921939},{0.648194, -0.580468, 2.921925},{0.623104, -0.574866, 2.921912},{0.598015, -0.569264, 2.921901},{0.572925, -0.563661, 2.921890},{0.547836, -0.558059, 2.921881},{0.522746, -0.552456, 2.921872},{0.497657, -0.546852, 2.921865},{0.472568, -0.541249, 2.921859},{0.447478, -0.535645, 2.921853},{0.422388, -0.530042, 2.921848},{0.397298, -0.524438, 2.921844},{0.372209, -0.518834, 2.921841},{0.347120, -0.513230, 2.921838},{0.322030, -0.507626, 2.921836},{0.296941, -0.502022, 2.921834},{0.271852, -0.496418, 2.921832},{0.247625, -0.491006, 2.921831},{0.224922, -0.485935, 2.921830},{0.203746, -0.481205, 2.921829},{0.184101, -0.476817, 2.921829},{0.165992, -0.472772, 2.921828},{0.149425, -0.469071, 2.921828},{0.134405, -0.465716, 2.921828},{0.120942, -0.462709, 2.921828},{0.109046, -0.460052, 2.921828},{0.098731, -0.457748, 2.921828},{0.090016, -0.455801, 2.921828},{0.082929, -0.454218, 2.921828},{0.077516, -0.453009, 2.921828},{0.073865, -0.452193, 2.921828},{0.073000, -0.452000, 2.921828}};
	std::vector<differentialVels> V = {{0.000000, 0.000000},{0.151220, -0.000000},{0.302441, -0.000000},{0.453661, -0.000001},{0.604881, -0.000001},{0.756101, -0.000003},{0.907322, -0.000005},{1.058542, -0.000007},{1.209762, -0.000011},{1.360983, -0.000016},{1.512203, -0.000023},{1.663423, -0.000032},{1.814644, -0.000044},{1.965864, -0.000060},{2.117084, -0.000079},{2.268305, -0.000105},{2.419525, -0.000137},{2.570745, -0.000179},{2.570745, -0.000218},{2.570745, -0.000262},{2.570745, -0.000312},{2.570745, -0.000370},{2.570745, -0.000436},{2.570745, -0.000511},{2.570745, -0.000594},{2.570745, -0.000686},{2.570745, -0.000787},{2.570745, -0.000894},{2.570745, -0.001007},{2.570745, -0.001122},{2.570745, -0.001233},{2.570745, -0.001337},{2.570745, -0.001426},{2.570745, -0.001495},{2.570745, -0.001539},{2.570745, -0.001554},{2.570745, -0.001538},{2.570745, -0.001494},{2.570745, -0.001424},{2.570745, -0.001334},{2.570745, -0.001230},{2.570745, -0.001119},{2.570745, -0.001004},{2.570745, -0.000891},{2.570745, -0.000784},{2.570745, -0.000683},{2.570745, -0.000591},{2.570745, -0.000508},{2.570745, -0.000434},{2.570745, -0.000368},{2.570745, -0.000310},{2.570745, -0.000260},{2.570745, -0.000216},{2.570745, -0.000178},{2.570745, -0.000145},{2.482399, -0.000113},{2.326268, -0.000085},{2.169785, -0.000064},{2.012892, -0.000047},{1.855522, -0.000034},{1.697580, -0.000024},{1.538947, -0.000017},{1.379464, -0.000011},{1.218892, -0.000007},{1.056904, -0.000004},{0.892969, -0.000002},{0.726172, -0.000001},{0.554714, -0.000000},{0.374084, -0.000000},{0.163670, 0.000000}};
	dt.move_differential_robot_vels(V, dt_);
}


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
	// compute sysid(fb);
	// std::vector<input_output> u_vs_x = sysid.fopdt_system_identification(200);
	// print_vector(u_vs_x);
    // motor_angle_mp_test(m1, m1_motor_constants, 78.497928, 4700.12687931 * 0.8f, 2.f * M_PI * 10);
    Intake.optical.set_led_pwm(75);
    Intake.optical.set_integration_time(10.0);
    
    while(true) {
	    if (dt.master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
	        Intake.intakeState = topScore;
			if (!redirector.is_extended()) {
				redirector.extend();
			}
	    }
        if (dt.master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
	        Intake.intakeState = midScore; 			
	    }
        if (dt.master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
	        Intake.intakeState = bottomScore; 			
	    }
        if (dt.master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
	        Intake.intakeState = intakeOnly;
			if (redirector.is_extended()) {
				redirector.retract();
			}		
	    }
        if (dt.master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
	        Intake.intakeState = intakeOff; 			
	    }
		if (dt.master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
	        descorer.toggle();
	    }
	 	Intake.update_intake_state(dt_);
	    dt.field_oriented_holonomic_control(dt_);
	    // dt.tank_drive_control(dt_);
        // dt.test_control(dt_);
        pros::delay(static_cast<int>(dt_*100.0));
    }
}
