// https://docs.platformio.org/en/latest/plus/unit-testing.html#unit-testing

#include <Arduino.h>
#include <unity.h>
#include <bobik.h>


// #define SERIAL_OUTPUT // if enabled run "pio test", Ctrl+C, "pio device monitor -b 500000" to see serial output
#define TEST_MOTORS


#define SIZE 500

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

void test_variance_fl(void)
{
  TestResult result;
  std_deviation(robot.caster_fl, &result);
  TEST_ASSERT_FLOAT_WITHIN(2.0, 3.0, result.std_dev);
  TEST_ASSERT_FLOAT_WITHIN(5.0, 0.0, result.average);
}
void test_variance_fr(void)
{
  TestResult result;
  std_deviation(robot.caster_fr, &result);
  TEST_ASSERT_FLOAT_WITHIN(2.0, 3.0, result.std_dev);
  TEST_ASSERT_FLOAT_WITHIN(5.0, 0.0, result.average);
}
void test_variance_r(void)
{
  TestResult result;
  std_deviation(robot.caster_r, &result);
  TEST_ASSERT_FLOAT_WITHIN(2.0, 3.0, result.std_dev);
  TEST_ASSERT_FLOAT_WITHIN(5.0, 0.0, result.average);
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
    delay(49);
  }
}

void test_sensor_based_motor_rotation_lf(void)
{
  TEST_ASSERT_EQUAL(true, false);
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
  RUN_TEST(test_variance_fl);
  RUN_TEST(test_variance_fr);
  RUN_TEST(test_variance_r);
  RUN_TEST(test_variance_fl);
  RUN_TEST(test_variance_fr);
  RUN_TEST(test_variance_r);
  RUN_TEST(test_variance_fl);
  RUN_TEST(test_variance_fr);
  RUN_TEST(test_variance_r);
  RUN_TEST(test_variance_fl);
  RUN_TEST(test_variance_fr);
  RUN_TEST(test_variance_r);

  #ifdef TEST_MOTORS
  RUN_TEST(test_sensor_based_motor_rotation_lf);
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
