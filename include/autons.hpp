#pragma once

const int SKILLS_DRIVE_SPEED = 75;
const int SKILLS_TURN_SPEED = 65;
const int DRIVE_SPEED = 83;
const int PARK_SPEED = 40;
const int TURN_SPEED = 65;
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

// skills
void skills_all_match_loader_park_control_zone();
void skills_and_mid();

// matches
void left_long_rush();
void left_long_middle();
void left_awp();
void left_large_awp();
void right_small_long_middle();
void right_long_middle();
void right_small_awp();
void right_large_awp();
void right_long_rush();
void move_slight();