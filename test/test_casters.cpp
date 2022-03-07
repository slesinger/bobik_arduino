// https://docs.platformio.org/en/latest/plus/unit-testing.html#unit-testing

#include <Arduino.h>
#include <unity.h> //https://github.com/ThrowTheSwitch/Unity
#include <bobik.h>

/*
* Enable required test suites here:
*/
// #define SERIAL_OUTPUT // if enabled run "pio test", Ctrl+C, "pio device monitor -b 500000" to see serial output
#define TEST_MOTORS
// #define TEST_ROTATION
#define TEST_DRIVE

#define SIZE 70
#define ROTATION_TOLERANCE 44  // 1 deg
#define GOOD_TIME_FOR_ROTATION 4

Bobik robot;

struct TestResult {
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
  for (int t=0; t<20*time_sec; t++)   //20Hz * 5 seconds, enough to rotate the caster
  {
    caster->execute();
    delay(1000/20);
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
    delay(1000/20);
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
  for (int t=0; t<20*5; t++)
  {
    ticks_actual_fl += robot.caster_fl->getDriveTicks();
    ticks_actual_fr += robot.caster_fr->getDriveTicks();
    ticks_actual_r += robot.caster_r->getDriveTicks();
    robot.caster_fl->execute();  //will zero tick count;
    robot.caster_fr->execute();  //will zero tick count;
    robot.caster_r->execute();  //will zero tick count;
    delay(1000/20);
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
  caster->setDriveTarget(DRIVE_TARGET);
  for (int t=0; t<20*10; t++) // total test run length
  {
    ticks_actual += caster->getDriveTicks();
    caster->execute();
    delay(1000/20);
  }
  TEST_ASSERT_INT32_WITHIN(50, DRIVE_TARGET, ticks_actual);
}

void test_drive_fwd(void)
{
  RUN_TEST(test_sensor_based_motor_rotation_all_zero);

  char buffer [128];
  uint32_t ticks_actual_fl = 0;
  uint32_t ticks_actual_fr = 0;
  uint32_t ticks_actual_r = 0;
  #define DRIVE_TARGET_FWD 240
  robot.caster_fl->setDriveTarget(DRIVE_TARGET_FWD);
  robot.caster_fr->setDriveTarget(DRIVE_TARGET_FWD);
  robot.caster_r->setDriveTarget(DRIVE_TARGET_FWD);

  for (int t=0; t<20*10; t++) // total test run length
  {
    ticks_actual_fl += robot.caster_fl->getDriveTicks();
    ticks_actual_fr += robot.caster_fr->getDriveTicks();
    ticks_actual_r  += robot.caster_r->getDriveTicks();
    robot.caster_fl->execute();
    robot.caster_fr->execute();
    robot.caster_r->execute();
    delay(1000/20);
  }

  snprintf(buffer, sizeof(buffer), "Ticks FL %lu   FR %lu   R %lu", ticks_actual_fl, ticks_actual_fr, ticks_actual_r);
  TEST_MESSAGE(buffer);
  #define BREAKING_PATH 60
  TEST_ASSERT_INT32_WITHIN(150, DRIVE_TARGET_FWD+BREAKING_PATH, ticks_actual_fl);  // TODO significantly improove read, change 150 to 30
  TEST_ASSERT_INT32_WITHIN(150, DRIVE_TARGET_FWD+BREAKING_PATH, ticks_actual_fr);
  TEST_ASSERT_INT32_WITHIN(150, DRIVE_TARGET_FWD+BREAKING_PATH, ticks_actual_r);
  Caster::stopAllCastersMotors();
}

void setup()
{
  // NOTE!!! Wait for >2 secs
  // if board doesn't support software reset via Serial.DTR/RTS
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Serial.begin(500000);
  delay(2000);
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
  Caster::stopAllCastersMotors();
  #endif // test_drive
  TEST_MESSAGE("FINISH UNIT TESTS");
  UNITY_END();
  #endif  // not serial_output

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
