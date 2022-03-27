#ifndef robot_config_h
#define robot_config_h

/*
*   This code is #included to Bobik::read_config
*/

// Note on ROS coordinate system (https://www.ros.org/reps/rep-0103.html)
// X forward, Y right*, Z up
// yaw (axis Z) zero position is pointing forward*, positive turning left
// * opposed to specification in REP103

    #define FPS 20

    // Base
    #define LEN_AB 0.457 // distance between twa caster axis
    #define LEN_AB_HALF LEN_AB / 2.0
    #define LEN_Cc LEN_AB * 1.732051 / 2.0 // height of same-side triangle, distance between point C and center of line c (AB)
    #define LEN_SC 2.0 / 3.0 * LEN_Cc  //distance between robot center (S - stred) and rear caster axis C
    #define LEN_Sc 1.0 / 3.0 * LEN_Cc  //distance between robot center (S - stred) and center of line c (AB)
    #define POS_A_x LEN_Sc
    #define POS_A_y -LEN_AB_HALF
    #define POS_B_x LEN_Sc
    #define POS_B_y LEN_AB_HALF
    #define POS_C_x -LEN_SC
    #define POS_C_y 0.0
    #define DEG_A M_PI / 3.0
    #define DEG_B M_PI / -3.0
    #define DEG_C M_PI

    #define CASTER_RAD2UNITS -8192.0 / M_PI
    #define CASTER_UNITS2RAD M_PI / -8192.0
    #define STOP_DUE_ROTATION_DIFF 0.18   // [rad]   >10deg difference in rotation requires to stop driving and wait for rotation to complete
    #define CASTER_DRIVE_MIN_SPEED 0.03
    #define CASTER_DRIVE_MAX_SPEED 0.4
    #define CASTER_METERS2TICKS (2*120) / (0.123 * M_PI)   // (falling + raising edge) * 120 holes / (D * PI)

    // Casters
    cfg.caster_fl.rotation_sensor.spi_cs_pin = 10;
    cfg.caster_fl.rotation_sensor.zero_position = 242;
    cfg.caster_fl.rotation_motor = {32, 34, 44};
    cfg.caster_fl.drive_motor = {28, 30, 4};
    cfg.caster_fl.drive_sensor_pin = 20;
    cfg.caster_fr.rotation_sensor.spi_cs_pin = 11;
    cfg.caster_fr.rotation_sensor.zero_position = 416;
    cfg.caster_fr.rotation_motor = {40, 42, 45};
    cfg.caster_fr.drive_motor = {36, 38, 5};
    cfg.caster_fr.drive_sensor_pin = 21;
    cfg.caster_r.rotation_sensor.spi_cs_pin =  12;
    cfg.caster_r.rotation_sensor.zero_position = 386;
    cfg.caster_r.rotation_motor = {41, 43, 46};
    cfg.caster_r.drive_motor = {37, 39, 6};
    cfg.caster_r.drive_sensor_pin = 19;

    // Head

    // Arm


#endif
