// https://docs.platformio.org/en/latest/plus/unit-testing.html#unit-testing

#include <Arduino.h>
#include <unity.h>

#include <AS5048A.h>
#define SIZE 500

AS5048A fl_caster_rotation_joint(10, 2592); // SPI cable select pin, zero angle value
AS5048A fr_caster_rotation_joint(11, 0);    // SPI cable select pin, zero angle value
AS5048A r_caster_rotation_joint(12, -768);  // SPI cable select pin, zero angle value

void init_as5048a()
{
  fl_caster_rotation_joint.init();
  fr_caster_rotation_joint.init();
  r_caster_rotation_joint.init();
}

float std_deviation(AS5048A *sensor)
{
  int data[SIZE];
  float sum = 0.0, mean, SD = 0.0;
  for (int i = 0; i < SIZE; i++)
  {
    data[i] = (*sensor).getRotation();
    sum += data[i];
  }
  mean = sum / SIZE;
  for (int i = 0; i < SIZE; i++)
  {
    SD += pow(data[i] - mean, 2);
  }
  return sqrt(SD / SIZE);
}

void test_variance_fl(void)
{
  float stddev = std_deviation(&fl_caster_rotation_joint);
  TEST_ASSERT_FLOAT_WITHIN(8.0, 10.0, stddev);
}
void test_variance_fr(void)
{
  float stddev = std_deviation(&fr_caster_rotation_joint);
  TEST_ASSERT_FLOAT_WITHIN(2.0, 3.0, stddev);
}
void test_variance_r(void)
{
  float stddev = std_deviation(&r_caster_rotation_joint);
  TEST_ASSERT_FLOAT_WITHIN(2.0, 3.0, stddev);
}

void test_output_rotation(void)
{
  for (int i = 0; i < SIZE; i++)
  {
    // Casters sensors
    int fl = fl_caster_rotation_joint.getRotation();
    int fr = fr_caster_rotation_joint.getRotation();
    int r = r_caster_rotation_joint.getRotation();
    Serial.print(fl);
    Serial.print('\t');
    Serial.print(fr);
    Serial.print('\t');
    Serial.println(r);
    delay(49);
  }
}

void setup()
{
  // NOTE!!! Wait for >2 secs
  // if board doesn't support software reset via Serial.DTR/RTS
  pinMode(13, OUTPUT);
  Serial.begin(500000);
  delay(2000);
  init_as5048a();
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
  RUN_TEST(test_variance_fl);
  RUN_TEST(test_variance_fl);
  RUN_TEST(test_variance_fl);
  RUN_TEST(test_variance_fl);
  RUN_TEST(test_variance_fl);
  RUN_TEST(test_variance_fl);
  UNITY_END();

  // test_output_rotation();
}

void loop()
{
  delay(1000);
}
