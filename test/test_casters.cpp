// https://docs.platformio.org/en/latest/plus/unit-testing.html#unit-testing

#include <Arduino.h>
#include <unity.h> //https://github.com/ThrowTheSwitch/Unity
#include <bobik.h>

/*
* Enable required test suites here:
*/
// #define SERIAL_OUTPUT // if enabled run "pio test", Ctrl+C, "pio device monitor -b 500000" to see serial output
#define TEST_MOTORS


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
  robot.caster_fl->pingRotationMotor();
  robot.caster_fr->pingRotationMotor();
  robot.caster_r->pingRotationMotor();

  RUN_TEST(test_sensor_based_motor_rotation_all_zero);
  RUN_TEST(test_sensor_based_motor_rotation_fl);
  RUN_TEST(test_sensor_based_motor_rotation_fr);
  RUN_TEST(test_sensor_based_motor_rotation_r);
  #endif
  UNITY_END();
  #endif

  #ifdef SERIAL_OUTPUT
  test_output_rotation();
  #endif
  digitalWrite(13, HIGH);
}

void loop()
{
  Serial.println("Done");
  delay(10000);
}
