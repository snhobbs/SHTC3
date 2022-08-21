#include <gtest/gtest.h>
#include <stdint.h>
#include <array>
#include <iostream>
#include <vector>

#include <SHTC3/SHTC3_I2CDriver.h>
TEST(shtc3, compiles) {
  SHTC3::Sensor sensor{};
  ASSERT_EQ(1, 1);
}

TEST(shtc3, Run_kBootup_to_kReadingData) {
  /*
   * Check that calling run correctly transitions states
   * */
  SHTC3::Sensor sensor{};
  ASSERT_EQ(sensor.get_state(), SHTC3::Sensor::State::kBootup);

  //  Sent a command
  sensor.Run();
  ASSERT_EQ(sensor.get_state(), SHTC3::Sensor::State::kWaiting);

  //  Will remain in reading state until all data is read or an error occurs
  sensor.Run();
  ASSERT_EQ(sensor.get_state(), SHTC3::Sensor::State::kReadingData);
}

TEST(shtc3, Run_Pushing_data_transitions_kReadingData_to_kWaiting) {
  SHTC3::Sensor sensor{};
  sensor.Run();
  sensor.Run();
  ASSERT_EQ(sensor.get_state(), SHTC3::Sensor::State::kReadingData);
  ASSERT_EQ(sensor.get_bytes_remaining(), sensor.kDataReadDataBytes);
  for (uint8_t ch=0; ch<sensor.kDataReadDataBytes; ch++) {
    EXPECT_EQ(sensor.get_state(), SHTC3::Sensor::State::kReadingData);
    sensor.PushData('a' + ch);
    EXPECT_EQ(sensor.get_bytes_remaining(), sensor.kDataReadDataBytes-ch-1);
  }
  EXPECT_EQ(sensor.get_bytes_remaining(), 0);

  //  requires an additional call to Run to transition
  sensor.Run();
  ASSERT_EQ(sensor.get_state(), SHTC3::Sensor::State::kWaiting);
}

TEST(shtc3, Run_CommandLength) {
  const size_t command_length = 7;
  SHTC3::Sensor sensor{};
  sensor.Run();

  size_t length = 0;
  while (sensor.OperationWaiting()) {
    I2COperation op{};
    sensor.GetNextOperation(op);
    length++;
  }
  EXPECT_EQ(length, command_length);
}

