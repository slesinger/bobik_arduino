// https://docs.platformio.org/en/latest/plus/unit-testing.html#unit-testing

#include <Arduino.h>
#include <unity.h> //https://github.com/ThrowTheSwitch/Unity
#include <bobik.h>
#include "robot_utils.h"


/*
 * Flags
*/
#define TEST_MOTORS
// #define SERIAL_OUTPUT // if enabled run "pio test", Ctrl+C, "pio device monitor -b 500000" to see serial output

/*
 * Enable required test suites here:
 */
// #define TEST_ROTATION
// #define TEST_DRIVE
#define TEST_HIGHLEVEL


#define SIZE 70
#define ROTATION_TOLERANCE 44 // 1 deg
#define GOOD_TIME_FOR_ROTATION 3

Bobik robot;

struct TestResult
{
  float average;
  float std_dev;
};

void std_deviation(Caster *caster, TestResult *res)
{
  int data[SIZE];
  float sum = 0.0, mean, SD = 0.0;
  for (int i = 0; i < SIZE; i++)
  {
    data[i] = (*caster).getRotation();
    sum += data[i];
  }
  mean = sum / SIZE;
  for (int i = 0; i < SIZE; i++)
  {
    SD += pow(data[i] - mean, 2);
  }
  res->average = mean;
  res->std_dev = sqrt(SD / SIZE);
  return;
}

void rotate_to_target(Caster *caster, int16_t target, int time_sec)
{
  caster->setRotationTarget(target);
  for (int t = 0; t < 20 * time_sec; t++) // 20Hz * 5 seconds, enough to rotate the caster
  {
    caster->execute();
    delay(1000 / 20);
  }
}

void test_variance_fl(void)
{
  TestResult result;
  std_deviation(robot.caster_fl, &result);
  TEST_ASSERT_FLOAT_WITHIN(0.9, 1.0, result.std_dev);
  // TEST_ASSERT_FLOAT_WITHIN(50.0, 0.0, result.average);
}
void test_variance_fr(void)
{
  TestResult result;
  std_deviation(robot.caster_fr, &result);
  TEST_ASSERT_FLOAT_WITHIN(0.9, 1.0, result.std_dev);
  // TEST_ASSERT_FLOAT_WITHIN(50.0, 0.0, result.average);
}
void test_variance_r(void)
{
  TestResult result;
  std_deviation(robot.caster_r, &result);
  TEST_ASSERT_FLOAT_WITHIN(0.9, 1.0, result.std_dev);
  // TEST_ASSERT_FLOAT_WITHIN(50.0, 0.0, result.average);
}

void test_output_rotation(void)
{
  for (int i = 0; i < SIZE; i++)
  {
    // Casters sensors
    int fl = robot.caster_fl->getRotation();
    int fr = robot.caster_fr->getRotation();
    int r = robot.caster_r->getRotation();
    Serial.print(fl);
    Serial.print('\t');
    Serial.print(fr);
    Serial.print('\t');
    Serial.println(r);
    delay(1000 / 20);
  }
}

void test_sensor_based_motor_rotation_all_zero(void)
{
  int16_t rotation_target = 0;
  TEST_MESSAGE("Set all casters to 0 rotation");
  rotate_to_target(robot.caster_fl, rotation_target, GOOD_TIME_FOR_ROTATION);
  int16_t rotation_actual = robot.caster_fl->getRotation();
  TEST_ASSERT_INT_WITHIN(ROTATION_TOLERANCE, rotation_target, rotation_actual);
  rotate_to_target(robot.caster_fr, rotation_target, GOOD_TIME_FOR_ROTATION);
  rotation_actual = robot.caster_fr->getRotation();
  TEST_ASSERT_INT_WITHIN(ROTATION_TOLERANCE, rotation_target, rotation_actual);
  rotate_to_target(robot.caster_r, rotation_target, GOOD_TIME_FOR_ROTATION);
  rotation_actual = robot.caster_r->getRotation();
  TEST_ASSERT_INT_WITHIN(ROTATION_TOLERANCE, rotation_target, rotation_actual);
}

void test_sensor_based_motor_rotation_fl(void)
{
  Caster *caster = robot.caster_fl;
  int16_t rotation_target = 0;
  rotate_to_target(caster, rotation_target, GOOD_TIME_FOR_ROTATION);
  int16_t rotation_actual = caster->getRotation();
  TEST_ASSERT_INT_WITHIN(ROTATION_TOLERANCE, rotation_target, rotation_actual);

  rotation_target = 4096;
  rotate_to_target(caster, rotation_target, GOOD_TIME_FOR_ROTATION);
  rotation_actual = caster->getRotation();
  TEST_ASSERT_INT_WITHIN(ROTATION_TOLERANCE, rotation_target, rotation_actual);

  rotation_target = 0;
  rotate_to_target(caster, rotation_target, GOOD_TIME_FOR_ROTATION);
  rotation_actual = caster->getRotation();
  TEST_ASSERT_INT_WITHIN(ROTATION_TOLERANCE, rotation_target, rotation_actual);
}

void test_sensor_based_motor_rotation_fr(void)
{
  Caster *caster = robot.caster_fr;
  int16_t rotation_target = 0;
  rotate_to_target(caster, rotation_target, GOOD_TIME_FOR_ROTATION);
  int16_t rotation_actual = caster->getRotation();
  TEST_ASSERT_INT_WITHIN(ROTATION_TOLERANCE, rotation_target, rotation_actual);

  rotation_target = 4096;
  rotate_to_target(caster, rotation_target, GOOD_TIME_FOR_ROTATION);
  rotation_actual = caster->getRotation();
  TEST_ASSERT_INT_WITHIN(ROTATION_TOLERANCE, rotation_target, rotation_actual);

  rotation_target = 0;
  rotate_to_target(caster, rotation_target, GOOD_TIME_FOR_ROTATION);
  rotation_actual = caster->getRotation();
  TEST_ASSERT_INT_WITHIN(ROTATION_TOLERANCE, rotation_target, rotation_actual);
}

void test_sensor_based_motor_rotation_r(void)
{
  Caster *caster = robot.caster_r;
  int16_t rotation_target = 0;
  rotate_to_target(caster, rotation_target, GOOD_TIME_FOR_ROTATION);
  int16_t rotation_actual = caster->getRotation();
  TEST_ASSERT_INT_WITHIN(ROTATION_TOLERANCE, rotation_target, rotation_actual);

  rotation_target = 4096;
  rotate_to_target(caster, rotation_target, GOOD_TIME_FOR_ROTATION);
  rotation_actual = caster->getRotation();
  TEST_ASSERT_INT_WITHIN(ROTATION_TOLERANCE, rotation_target, rotation_actual);

  rotation_target = 0;
  rotate_to_target(caster, rotation_target, GOOD_TIME_FOR_ROTATION);
  rotation_actual = caster->getRotation();
  TEST_ASSERT_INT_WITHIN(ROTATION_TOLERANCE, rotation_target, rotation_actual);
}

void test_move_fwd_5seconds(void)
{
  RUN_TEST(test_sensor_based_motor_rotation_all_zero);
  uint32_t ticks_actual_fl = 0;
  uint32_t ticks_actual_fr = 0;
  uint32_t ticks_actual_r = 0;
  // robot.caster_fl->setDriveEffort(200); //this methods is removed, replace it by pingDrive
  // robot.caster_fr->setDriveEffort(200);
  // robot.caster_r->setDriveEffort(200);
  for (int t = 0; t < 20 * 5; t++)
  {
    ticks_actual_fl += robot.caster_fl->getDriveTicks();
    ticks_actual_fr += robot.caster_fr->getDriveTicks();
    ticks_actual_r += robot.caster_r->getDriveTicks();
    robot.caster_fl->execute(); // will zero tick count;
    robot.caster_fr->execute(); // will zero tick count;
    robot.caster_r->execute();  // will zero tick count;
    delay(1000 / 20);
  }
  // robot.caster_fl->setDriveEffort(0);
  // robot.caster_fr->setDriveEffort(0);
  // robot.caster_r->setDriveEffort(0);

  TEST_ASSERT_INT32_WITHIN(500, 2000, ticks_actual_fl);
  TEST_ASSERT_INT32_WITHIN(500, 2000, ticks_actual_fr);
  TEST_ASSERT_INT32_WITHIN(500, 2000, ticks_actual_r);
}

void test_drive_1000ticks_r(void)
{
  Caster *caster = robot.caster_r;
  uint32_t ticks_actual = 0;
  #define DRIVE_TARGET 1000
  caster->setDriveTarget(DRIVE_TARGET, false);
  for (int t = 0; t < 20 * 10; t++) // total test run length
  {
    ticks_actual += caster->getDriveTicks();
    caster->execute();
    delay(1000 / 20);
  }
  TEST_ASSERT_INT32_WITHIN(50, DRIVE_TARGET, ticks_actual);
}

void test_drive_fwd(void)
{
  RUN_TEST(test_sensor_based_motor_rotation_all_zero);

  char buffer[128];
  uint32_t ticks_actual_fl = 0;
  uint32_t ticks_actual_fr = 0;
  uint32_t ticks_actual_r = 0;
#define DRIVE_TARGET_FWD 240 * 3
  robot.caster_fl->setDriveTarget(DRIVE_TARGET_FWD, false);
  robot.caster_fr->setDriveTarget(DRIVE_TARGET_FWD, false);
  robot.caster_r->setDriveTarget(DRIVE_TARGET_FWD, false);

  for (int t = 0; t < 20 * 10; t++) // total test run length
  {
    ticks_actual_fl += robot.caster_fl->getDriveTicks();
    ticks_actual_fr += robot.caster_fr->getDriveTicks();
    ticks_actual_r += robot.caster_r->getDriveTicks();
    robot.execute();
    delay(1000 / 20);
  }

  snprintf(buffer, sizeof(buffer), "Ticks FL %lu   FR %lu   R %lu", ticks_actual_fl, ticks_actual_fr, ticks_actual_r);
  TEST_MESSAGE(buffer);
#define BREAKING_PATH 60
  TEST_ASSERT_INT32_WITHIN(150, DRIVE_TARGET_FWD + BREAKING_PATH, ticks_actual_fl); // TODO significantly improove read, change 150 to 30
  TEST_ASSERT_INT32_WITHIN(150, DRIVE_TARGET_FWD + BREAKING_PATH, ticks_actual_fr);
  TEST_ASSERT_INT32_WITHIN(150, DRIVE_TARGET_FWD + BREAKING_PATH, ticks_actual_r);
  Bobik::stopAllCastersMotors();
}

int f2i(float f)
{
  return round(f * 1000);
}

// Run test with python visualiser
// pio test | python test/plot_base.py
void hl_casters(int num_frames, float x, float y, float g, int enable_motors)
{
  char buffer[128];
  float dbg[12];

  // simulate arduino loop()
  for (int t = 0; t < num_frames; t++) // total test run length
  {
    robot.setCmdVel(x, y, g); // forward, strafe left, rotate left
    robot.getCmdVelDebug(dbg);
    int dept_fl = robot.caster_fl->getDriveTicksDept();
    int dept_fr = robot.caster_fr->getDriveTicksDept();
    int dept_r = robot.caster_r->getDriveTicksDept();
    snprintf(buffer, sizeof(buffer), "Base config;%d; %d;%d;%d; %d;%d;%d;%d; %d;%d;%d;%d; %d;%d;%d;%d | %d;%d;%d", t, f2i(x), f2i(y), f2i(g),
             f2i(dbg[0]), f2i(dbg[1]), f2i(dbg[2]), f2i(dbg[3]), f2i(dbg[4]), f2i(dbg[5]), f2i(dbg[6]), f2i(dbg[7]), f2i(dbg[8]), f2i(dbg[9]), f2i(dbg[10]), f2i(dbg[11]), dept_fl, dept_fr, dept_r);
    TEST_MESSAGE(buffer);

    if (enable_motors == true)
      robot.execute();
    delay(1000 / 20);
  }
}

void test_hl_scenario()
{
  int motors = false;
#ifdef TEST_MOTORS
  motors = true;
#endif
  char buffer[128];
  snprintf(buffer, sizeof(buffer), "Base config hdr;frame; X;Y;gamma; flx;fly;flg;flspd; frx;fry;frg;frspd; rx;ry;rg;rspd");
  TEST_MESSAGE(buffer);
  // frames, cmd_vel x, y, gamma
  
  // drive speed test
  // hl_casters(20*1, 0.1, 0.0, 0.0, motors);
  // hl_casters(20*5, 0.2, 0.0, 0.0, motors);
  // hl_casters(20*1, 0.3, 0.0, 0.0, motors);
  // hl_casters(20*1, 0.4, 0.0, 0.0, motors);
  // hl_casters(20*1, 0.3, 0.0, 0.0, motors);
  // hl_casters(20*1, 0.2, 0.0, 0.0, motors);
  // hl_casters(20*1, 0.1, 0.0, 0.0, motors);
  // hl_casters(20*1, 0.0, 0.0, 0.0, motors);
  
  // drive speed test backwards
  // hl_casters(20*1, -0.1, 0.0, 0.0, motors);
  // hl_casters(20*1, -0.2, 0.0, 0.0, motors);
  // hl_casters(20*1, -0.3, 0.0, 0.0, motors);
  // hl_casters(20*1, -0.4, 0.0, 0.0, motors);
  // hl_casters(20*1, -0.3, 0.0, 0.0, motors);
  // hl_casters(20*1, -0.2, 0.0, 0.0, motors);
  // hl_casters(20*1, -0.1, 0.0, 0.0, motors);
  // hl_casters(20*1, 0.0, 0.0, 0.0, motors);
  
  // fwd - strafe left - bck - strafe right - fwd for init
  // hl_casters(20 * 1, 0.4, 0.0, 0.0, motors);
  // hl_casters(20 * 4, 0.0, -0.2, 0.0, motors);
  // hl_casters(20 * 4, -0.2, 0.0, 0.0, motors);
  // hl_casters(20 * 4, 0.0, 0.3, 0.0, motors);

  // fwd + alternate slight left-right rotations
  // hl_casters(20 * 1, 0.4, 0.0, 0.0, motors);
  // hl_casters(20 * 4, 0.4, 0.0, 0.5, motors);
  // hl_casters(20 * 4, 0.4, 0.0, -0.5, motors);
  // hl_casters(20 * 1, 0.4, 0.0, 0.0, motors);

  // fwd, rot left on spot, rot right on spot, fwd
  // hl_casters(20 * 1, 0.2, 0.0, 0.0, motors);
  // hl_casters(20 * 4, 0.0, 0.0, 1.0, motors);
  // hl_casters(1, 0.0, 0.0, 0.0, motors);
  // hl_casters(20 * 4, 0.0, 0.0, -1.0, motors);

  // full fwd + onboard rotation left, decrease fwd animation
  // hl_casters(20 * 1, 0.4, 0.0, 0.1, motors);
  // hl_casters(20 * 1, 0.4, 0.0, 0.2, motors);
  // hl_casters(20 * 1, 0.4, 0.0, 0.3, motors);
  // hl_casters(20 * 1, 0.4, 0.0, 0.4, motors);
  // hl_casters(20 * 1, 0.4, 0.0, 0.5, motors);
  // hl_casters(20 * 1, 0.4, 0.0, 0.6, motors);
  // hl_casters(20 * 1, 0.4, 0.0, 0.7, motors);
  // hl_casters(20 * 1, 0.4, 0.0, 0.8, motors);
  // hl_casters(20 * 1, 0.4, 0.0, 0.9, motors);
  // hl_casters(20 * 1, 0.4, 0.0, 1.0, motors);
  // hl_casters(20 * 1, 0.4, 0.0, 1.5, motors);
  // hl_casters(20 * 1, 0.4, 0.0, 2.0, motors);
  // hl_casters(20 * 1, 0.4, 0.0, 2.5, motors);
  // hl_casters(20 * 1, 0.4, 0.0, 3.0, motors);
  // hl_casters(20 * 1, 0.4, 0.0, 3.5, motors);
  // hl_casters(20 * 1, 0.2, 0.0, 3.5, motors);
  // hl_casters(20 * 1, 0.0, 0.0, 3.5, motors);

  // slow fwd and fast rotation (like PR2 demo)
  // hl_casters(20 * 10, 0.15, 0.15, 1.5, motors);  //not able to judge if this works while on test bench

  // stop
  hl_casters(20 * 2, 0.0, 0.0, 0.0, motors); 
}

void test_base_simplify_rad()
{
  UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.0001, 0, robot.simplify_rad(0.0), 1, "0");
  UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.0001, M_PI, robot.simplify_rad(M_PI), 2, "M_PI");
  UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.0001, -M_PI, robot.simplify_rad(-M_PI), 3, "-M_PI");
  UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.0001, M_PI / 3, robot.simplify_rad(M_PI / 3), 4, "M_PI/3");
  UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.0001, -M_PI / 3, robot.simplify_rad(-M_PI / 3), 5, "-M_PI/3");
  UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.0001, -M_PI * 2.0 / 3, robot.simplify_rad(-M_PI * 2.0 / 3), 6, "-M_PI*2.0/3");
  UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.0001, -M_PI * 0.1, robot.simplify_rad(M_PI * 1.9), 7, "M_PI*1.9");
  UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.0001, M_PI * 0.1, robot.simplify_rad(-M_PI * 1.9), 8, "-M_PI*1.9");
  UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.0001, M_PI * 0.5, robot.simplify_rad(M_PI * 6.5), 9, "M_PI*6.5");
  UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.0001, -M_PI * 0.5, robot.simplify_rad(-M_PI * 6.5), 10, "-M_PI*6.5");
  UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.0001, -M_PI * 0.1, robot.simplify_rad(M_PI * 3.9), 11, "M_PI*3.9");
  UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.0001, M_PI * 0.1, robot.simplify_rad(-M_PI * 3.9), 11, "-M_PI*3.9");
}

void test_optimize_rotation()
{
  // see docs/caster-rotation-angles-optimization.ods
  float rad = 0.0;
  UNITY_TEST_ASSERT_EQUAL_INT(1, robot.optimize_rotation(0.0, &rad), 1, "1"); // 0 ~ 0
  UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.0001, 0.0, rad, 1, "1a");
  rad = M_PI * 0.499;
  UNITY_TEST_ASSERT_EQUAL_INT(1, robot.optimize_rotation(0.0, &rad), 1, "2"); // 0 ~ 89
  UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.0001, M_PI * 0.499, rad, 1, "2a");
  rad = M_PI * 0.51;
  UNITY_TEST_ASSERT_EQUAL_INT(-1, robot.optimize_rotation(0.0, &rad), 1, "3"); // 0 ~ 91
  UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.0001, -M_PI * (1 - 0.51), rad, 1, "3a");
  rad = M_PI * 0.99;
  UNITY_TEST_ASSERT_EQUAL_INT(-1, robot.optimize_rotation(0.0, &rad), 1, "4"); // 0 ~ 179
  UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.0001, -M_PI * (1 - 0.99), rad, 1, "4a");
  rad = M_PI * 1.0;
  UNITY_TEST_ASSERT_EQUAL_INT(-1, robot.optimize_rotation(0.0, &rad), 1, "5"); // 0 ~ 180
  UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.0001, 0.0, rad, 1, "5a");
  rad = -M_PI * 0.499;
  UNITY_TEST_ASSERT_EQUAL_INT(1, robot.optimize_rotation(0.0, &rad), 1, "6"); // 0 ~ -89
  UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.0001, -M_PI * 0.499, rad, 1, "6a");
  rad = -M_PI * 0.51;
  UNITY_TEST_ASSERT_EQUAL_INT(-1, robot.optimize_rotation(0.0, &rad), 1, "7"); // 0 ~ -91
  UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.0001, M_PI * (1 - 0.51), rad, 1, "7a");
  rad = -M_PI * 0.99;
  UNITY_TEST_ASSERT_EQUAL_INT(-1, robot.optimize_rotation(0.0, &rad), 1, "8"); // 0 ~ -179
  UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.0001, M_PI * (1 - 0.99), rad, 1, "8a");
  rad = -M_PI;
  UNITY_TEST_ASSERT_EQUAL_INT(-1, robot.optimize_rotation(0.0, &rad), 1, "9"); // 0 ~ -180
  UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.0001, 0.0, rad, 1, "9a");

  rad = (-0.05) * M_PI;
  UNITY_TEST_ASSERT_EQUAL_INT(1, robot.optimize_rotation(0.4 * M_PI, &rad), 2, "10"); // 80 ~ -5 (<90)
  UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.0001, (-0.05) * M_PI, rad, 2, "10a");
  rad = (-0.0) * M_PI;
  UNITY_TEST_ASSERT_EQUAL_INT(1, robot.optimize_rotation(0.4 * M_PI, &rad), 2, "11"); // 80 ~ 0 (<90)
  UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.0001, (0.0) * M_PI, rad, 2, "11a");
  rad = (-0.1) * M_PI;
  UNITY_TEST_ASSERT_EQUAL_INT(1, robot.optimize_rotation(0.4 * M_PI, &rad), 2, "12"); // 80 ~ -10 (=90)
  UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.0001, (-0.1) * M_PI, rad, 2, "12a");
  rad = (-0.4) * M_PI;
  UNITY_TEST_ASSERT_EQUAL_INT(-1, robot.optimize_rotation(0.4 * M_PI, &rad), 2, "13"); // 80 ~ -80 (>>90)
  UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.0001, (1.0 - 0.4) * M_PI, rad, 2, "13a");
  rad = (-0.2) * M_PI;
  UNITY_TEST_ASSERT_EQUAL_INT(1, robot.optimize_rotation(0.4 * M_PI, &rad), 2, "14"); // 80 ~ -80 (>90)  !too close to overflow
  UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.0001, (-0.2) * M_PI, rad, 2, "14a");

  rad = (0.2) * M_PI;
  UNITY_TEST_ASSERT_EQUAL_INT(1, robot.optimize_rotation(0.6 * M_PI, &rad), 2, "15"); // 100 ~ 20 (<90)
  UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.0001, (0.2) * M_PI, rad, 2, "15a");
  rad = (-0.0) * M_PI;
  UNITY_TEST_ASSERT_EQUAL_INT(1, robot.optimize_rotation(0.6 * M_PI, &rad), 2, "16"); // 100 ~ 0 (>90) !would overflow
  UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.0001, (0.0) * M_PI, rad, 2, "16a");
  rad = (0.1) * M_PI;
  UNITY_TEST_ASSERT_EQUAL_INT(1, robot.optimize_rotation(0.6 * M_PI, &rad), 2, "17"); // 100 ~ 10 (=90)
  UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.0001, (0.1) * M_PI, rad, 2, "17a");
  rad = (-0.4) * M_PI;
  UNITY_TEST_ASSERT_EQUAL_INT(-1, robot.optimize_rotation(0.6 * M_PI, &rad), 2, "18"); // 100 ~ -80 (>>90)
  UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.0001, (1.0 - 0.4) * M_PI, rad, 2, "18a");
  rad = (1.0) * M_PI;
  UNITY_TEST_ASSERT_EQUAL_INT(-1, robot.optimize_rotation(0.6 * M_PI, &rad), 2, "19"); // 100 ~ 180 (<90)  !would overflow
  UNITY_TEST_ASSERT_DOUBLE_WITHIN(0.0001, (1.0 - 1.0) * M_PI, rad, 2, "19a");
}

void test_map_cut()
{
  int16_t res;
  res = RobotUtils::map_cut(-80000, 1, 12, 90, 255);
  UNITY_TEST_ASSERT_EQUAL_INT(90, res, 1, "map_cut");
  res = RobotUtils::map_cut(0, 1, 12, 90, 255);
  UNITY_TEST_ASSERT_EQUAL_INT(90, res, 1, "map_cut");
  res = RobotUtils::map_cut(1, 1, 12, 90, 255);
  UNITY_TEST_ASSERT_EQUAL_INT(90, res, 1, "map_cut");
  res = RobotUtils::map_cut(0, 0, 12, 0, 255);
  UNITY_TEST_ASSERT_EQUAL_INT(0, res, 1, "map_cut");
  res = RobotUtils::map_cut(10, 1, 12, 90, 255);
  UNITY_TEST_ASSERT_EQUAL_INT(225, res, 1, "map_cut");
  res = RobotUtils::map_cut(12, 1, 12, 90, 255);
  UNITY_TEST_ASSERT_EQUAL_INT(255, res, 1, "map_cut");
  res = RobotUtils::map_cut(280, 1, 12, 90, 255);
  UNITY_TEST_ASSERT_EQUAL_INT(255, res, 1, "map_cut");
}

void setup()
{
  // NOTE!!! Wait for >2 secs
  // if board doesn't support software reset via Serial.DTR/RTS
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Serial.begin(500000);
  delay(1000);
#ifndef SERIAL_OUTPUT
  UNITY_BEGIN();
  TEST_MESSAGE("START UNIT TESTS");

#ifdef TEST_ROTATION
  TEST_MESSAGE("ROTATION");
#ifdef TEST_MOTORS
#endif
  RUN_TEST(test_variance_fl);
  RUN_TEST(test_variance_fr);
  RUN_TEST(test_variance_r);
  RUN_TEST(test_variance_fl);
  RUN_TEST(test_variance_fr);
  RUN_TEST(test_variance_r);

#ifdef TEST_MOTORS
  // a little swing there and back, no position reading
  // robot.caster_fl->pingRotationMotor();
  // robot.caster_fr->pingRotationMotor();
  // robot.caster_r->pingRotationMotor();

  RUN_TEST(test_sensor_based_motor_rotation_all_zero);
// RUN_TEST(test_sensor_based_motor_rotation_fl);
// RUN_TEST(test_sensor_based_motor_rotation_fr);
// RUN_TEST(test_sensor_based_motor_rotation_r);
#endif // test_motors
#endif // test_rotation

#ifdef TEST_DRIVE
  TEST_MESSAGE("DRIVE");
  // robot.caster_fl->pingDriveMotor();
  // robot.caster_fr->pingDriveMotor();
  // robot.caster_r->pingDriveMotor();
  // RUN_TEST(test_move_fwd_5seconds);  // 10 seconds, try to move wheels by hand
  // RUN_TEST(test_drive_1000ticks_r); // with PID
  RUN_TEST(test_drive_fwd);
  Bobik::stopAllCastersMotors();
#endif // test_drive

#ifdef TEST_HIGHLEVEL
#ifdef TEST_MOTORS
  hl_casters(20 * 1, 0.03, 0.0, 0.0, true);
#endif
  RUN_TEST(test_base_simplify_rad);
  RUN_TEST(test_optimize_rotation);
  RUN_TEST(test_map_cut);
  RUN_TEST(test_hl_scenario);
#ifdef TEST_MOTORS
  Bobik::stopAllCastersMotors();
#endif
#endif
  TEST_MESSAGE("FINISH UNIT TESTS");
  UNITY_END();
#endif // not serial_output

#ifdef SERIAL_OUTPUT
  test_output_rotation();
#endif
  digitalWrite(13, HIGH);
}

void loop()
{
  TEST_MESSAGE("Pause in loop");
  delay(10000);
}
