/*
 * Copyright 2020 ElectroOptical Innovations, LLC
 */

#pragma once

#include <I2CBus/I2CInterface.h>
#include <Utilities/Crc.h>
#include <Utilities/TypeConversion.h>
#include <algorithm>
#include <array>
#include <cstdint>

namespace SHTC3 {
enum class MeasurementType : uint16_t {
  kNormalClockStretchTFirst = 0x7CA2,
  kNormalClockStretchRHFirst = 0x5C24,
  kLowPowerClockStretchTFirst = 0x6458,
  kLowPowerClockStretchRHFirst = 0x44de,
  kNormalNoClockStretchTFirst = 0x7866,
  kNormalNoClockStretchRHFirst = 0x58e0,
  kLowPowerNoClockStretchTFirst = 0x609c,
  kLowPowerNoClockStretchRHFirst = 0x401a,
};
enum class Command : uint16_t {
  kSoftwareReset = 0x805D,
  kReadId = 0xefc8,
  kSleep = 0xB098,
  kWakeup = 0x3517,
  kMeasurementNormalClockStretchTFirst =
      static_cast<uint16_t>(MeasurementType::kNormalClockStretchTFirst),
  kMeasurementNormalClockStretchRHFirst =
      static_cast<uint16_t>(MeasurementType::kNormalClockStretchRHFirst),
  kMeasurementLowPowerClockStretchTFirst =
      static_cast<uint16_t>(MeasurementType::kLowPowerClockStretchTFirst),
  kMeasurementLowPowerClockStretchRHFirst =
      static_cast<uint16_t>(MeasurementType::kLowPowerClockStretchRHFirst),

  kMeasurementNormalNoClockStretchTFirst =
      static_cast<uint16_t>(MeasurementType::kNormalNoClockStretchTFirst),
  kMeasurementNormalNoClockStretchRHFirst =
      static_cast<uint16_t>(MeasurementType::kNormalNoClockStretchRHFirst),
  kMeasurementLowPowerNoClockStretchTFirst =
      static_cast<uint16_t>(MeasurementType::kLowPowerNoClockStretchTFirst),
  kMeasurementLowPowerNoClockStretchRHFirst =
      static_cast<uint16_t>(MeasurementType::kLowPowerNoClockStretchRHFirst),
  kNoCommand = 0x0000,
};
const constexpr uint8_t kSlaveAddress = 0x70;
const constexpr uint8_t kSlaveRead = I2c::MakeSlaveReadAddress(kSlaveAddress);
const constexpr uint8_t kSlaveWrite = I2C::MakeSlaveWriteAddress(kSlaveAddress);
static_assert(kSlaveRead == (0x70 << 1) + 1);
static_assert(kSlaveWrite == (0x70 << 1));

inline constexpr uint8_t crc8(const uint8_t *const buffer,
                              const std::size_t data_length) {
  const Utilities::Crc8Setting settings{0x31, 0xff, 0x00, false, false};
  return Utilities::crc_uint8(buffer, data_length, settings);
}

static_assert(crc8(std::array<uint8_t, 2>{0xBE, 0xEF}.data(), 2) == 0x92);
static_assert(crc8(std::array<uint8_t, 1>{0x00}.data(), 1) == 0xAC);
static_assert(crc8(std::array<uint8_t, 2>{101, 228}.data(), 2) == 166);
static_assert(crc8(std::array<uint8_t, 2>{63, 203}.data(), 2) == 7);

class Calculator {
  static const constexpr std::size_t kNumberOfAdcBits = 16;
  static const constexpr std::size_t kMaxAdcValue = (1<<kNumberOfAdcBits) - 1;
  static const constexpr int32_t kMicroConversionFactor = static_cast<int32_t>(1e6);

 public:
  static int32_t
  ConvertHumidityReadingToMicroPercent(const int32_t humidity_reading) {
	const constexpr auto conversion_factor = static_cast<int32_t>(Utilities::round(100 * static_cast<double>(kMicroConversionFactor) / static_cast<double>(kMaxAdcValue)));
	static_assert(conversion_factor == 1526);
    return (conversion_factor * humidity_reading);
  }
  static int32_t
  ConvertTemperatureReadingToMicroCelsius(const int32_t temp_reading) {
	const constexpr auto conversion_factor = static_cast<int32_t>(Utilities::round(175 * static_cast<double>(kMicroConversionFactor)) / static_cast<double>(kMaxAdcValue));
	static_assert(conversion_factor == 2670);
	return -45 * kMicroConversionFactor + conversion_factor * temp_reading;
  }
};

class Sensor : public I2CDeviceBase {
  /*
   * Sensiron
   * SHTC3
   * 400kHz
   *
   * Reads always include both the temp and pressure
   * in definable order
   * Reads after a measurement command are data a1, data b1, crc, data a2, data
   * b2, crc MSB Data
   * */
  static const uint32_t kAdcDataBytes = 2;
  static const constexpr uint32_t kDataReadDataBytes =
      2 * (kAdcDataBytes + 1); //  2 data bytes + 1 crc

  enum class State { kBootup, kReadingData, kWaiting };

  State state_ = State::kBootup;
  uint16_t humidity_reading_ = 0;
  uint16_t temperature_reading_ = 0;
  int32_t humidity_ = 0;
  int32_t temperature_ = 0;
  int32_t byte_count_ = 0;
  bool crc_failed_flag_ = false;
  std::array<uint8_t, kDataReadDataBytes> data_in_{};

 private:
  void SendCommand(Command command) {
    InsertOperation({I2COperationType::kWrite, kSlaveWrite});
    InsertOperation({I2COperationType::kStart});
    InsertOperation({I2COperationType::kWrite,
                     Utilities::GetByte(static_cast<uint16_t>(command), 1)});
    InsertOperation({I2COperationType::kContinue});
    InsertOperation({I2COperationType::kWrite,
                     Utilities::GetByte(static_cast<uint16_t>(command), 0)});
    InsertOperation({I2COperationType::kContinue});
    InsertOperation({I2COperationType::kStop});
  }

  void StartRead(void) {
    InsertOperation({I2C::OperationType::kWrite, kSlaveRead});
    InsertOperation({I2C::OperationType::kStart});

    for (std::size_t i = 0; i < kDataReadDataBytes; i++) {
      InsertOperation({I2C::OperationType::kRead});
      InsertOperation({I2C::OperationType::kContinue});
    }

    InsertOperation({I2C::OperationType::kStop});
  }

  void SetupRead(void) {
    CleanupRead();
    byte_count_ = kDataReadDataBytes;
    state_ = State::kReadingData;
  }

  void CleanupRead(void) {
    std::fill(data_in_.begin(), data_in_.end(), 0);
  }

  void SendReset(void) { SendCommand(Command::kSoftwareReset); }

  /*
   * Check CRC bytes, if correct set readings, otherwise keep last reading, set
   * error flag
   * */
  void ProcessData(void) {
    // Check CRC
    if ((data_in_[kAdcDataBytes] != crc8(data_in_.begin(), kAdcDataBytes)) ||
        (data_in_.back() !=
         crc8(&data_in_[kAdcDataBytes + 1], kAdcDataBytes))) {
      crc_failed_flag_ = true;
    } else {
      /* CRCs are valid, update the readings */
      crc_failed_flag_ = false;
      temperature_reading_ =
          Utilities::Make_MSB_IntegerTypeFromU8Array<uint16_t>(
              data_in_.data(), sizeof(uint16_t));
      humidity_reading_ = Utilities::Make_MSB_IntegerTypeFromU8Array<uint16_t>(
          &data_in_[kAdcDataBytes + 1], sizeof(uint16_t));
    }
  }

 public:
  virtual void Run(void) {
    switch (state_) {
    case (State::kBootup):
      //  Reset Device
      Reset();
      SendCommand(Command::kWakeup);
      state_ = State::kWaiting;
      break;
    case (State::kWaiting):
      //  Start read
      SetupRead();
      SendCommand(Command::kWakeup);
      // SendCommand(Command::kMeasurementLowPowerNoClockStretchTFirst);
      SendCommand(Command::kMeasurementLowPowerClockStretchTFirst);
      StartRead();
      state_ = State::kReadingData;
      break;
    case (State::kReadingData):
      //  Check for finished
      if (byte_count_ == 0) {
        ProcessData();
        state_ = State::kWaiting;
      }
      break;
    default:
      assert(0);
      break;
    }
  }

  virtual void Reset(void) {
    // SendReset();
    state_ = State::kWaiting;
    CleanupRead();
    ResetOperations();
  }

  virtual void PushData(uint8_t data) {
    if ((state_ == State::kReadingData) && (byte_count_ > 0)) { //  valid state
      data_in_[data_in_.size() - byte_count_] = data;
      byte_count_--;
    } else {
      //  invalid push data
      assert(0);
      Reset();
    }
  }

  uint32_t get_humidity_reading(void) const { return humidity_reading_; }
  uint32_t get_temperature_reading(void) const { return temperature_reading_; }
  int32_t get_humidity_micro_percent(void) const {
    return Calculator::ConvertHumidityReadingToMicroPercent(
        get_humidity_reading());
  }
  int32_t get_temperature_micro_celsius(void) const {
    return Calculator::ConvertTemperatureReadingToMicroCelsius(
        get_temperature_reading());
  }

  Sensor(void) {}
};
}  // namespace SHTC3