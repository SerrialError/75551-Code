#pragma once

const int DRIVE_SPEED = 65;
const int TURN_SPEED = 60;
const int SWING_SPEED = 70;

void default_constants();
void drive_example();
void turn_example();
void drive_and_turn();
void wait_until_change_speed();
void swing_example();
void motion_chaining();
void combining_movements();
void interfered_example();
void odom_drive_example();
void odom_pure_pursuit_example();
void odom_pure_pursuit_wait_until_example();
void odom_boomerang_example();
void odom_boomerang_injected_pure_pursuit_example();
void measure_offsets();

// custom autons
void skills_deload_left_park();
void skills_park_only();
void skills_all_match_loader_park();
void skills_all_park_double_clear();
void redLeft_1_side_long_goal();
void redRight_1_side_long_goal();
void blueLeft_1_side_long_goal();
void blueRight_1_side_long_goal();
void redRight_sawp();
void blueRight_sawp();
void move_slight();
