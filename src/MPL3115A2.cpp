#include <octo/MPL3115A2.hpp>
#include <stdexcept>          // std::runtime_error
#include <cmath>              // log2


namespace Octo {

  /// Count the number of trailing 0 bits
  constexpr uint8_t cntt0(uint8_t value) {
    // Only one return statement is acceptable until C++14
    return \
        (value & 0x01) ? 0
      : (value & 0x02) ? 1
      : (value & 0x04) ? 2
      : (value & 0x08) ? 3
      : (value & 0x10) ? 4
      : (value & 0x20) ? 5
      : (value & 0x40) ? 6
      : (value & 0x80) ? 7
      : 8;
  }


  // Pimpl idiom class
  class MPL3115A2::Impl {
    public:
      // Configuration options
      static const uint8_t MEASUREMENT_ATTEMPTS = 50;

      // Constants from datasheet
      static const uint8_t SLAVE_ADDRESS = 0x60;
      static const uint8_t DEVICE_ID     = 0xC4;

      // Registers
      enum Register: uint8_t {
        STATUS          = 0x00,
        OUT_P_MSB       = 0x01,
        OUT_P_CSB       = 0x02,
        OUT_P_LSB       = 0x03,
        OUT_T_MSB       = 0x04,
        OUT_T_LSB       = 0x05,
        DR_STATUS       = 0x06,
        OUT_P_DELTA_MSB = 0x07,
        OUT_P_DELTA_CSB = 0x08,
        OUT_P_DELTA_LSB = 0x09,
        OUT_T_DELTA_MSB = 0x0A,
        OUT_T_DELTA_LSB = 0x0B,
        WHO_AM_I        = 0x0C,
        F_STATUS        = 0x0D,
        F_DATA          = 0x0E,
        F_SETUP         = 0x0F,
        TIME_DLY        = 0x10,
        SYSMOD          = 0x11,
        INT_SOURCE      = 0x12,
        PT_DATA_CFG     = 0x13,
        BAR_IN_MSB      = 0x14,
        BAR_IN_LSB      = 0x15,
        P_TGT_MSB       = 0x16,
        P_TGT_LSB       = 0x17,
        T_TGT           = 0x18,
        P_WND_MSB       = 0x19,
        P_WND_LSB       = 0x1A,
        T_WND           = 0x1B,
        P_MIN_MSB       = 0x1C,
        P_MIN_CSB       = 0x1D,
        P_MIN_LSB       = 0x1E,
        T_MIN_MSB       = 0x1F,
        T_MIN_LSB       = 0x20,
        P_MAX_MSB       = 0x21,
        P_MAX_CSB       = 0x22,
        P_MAX_LSB       = 0x23,
        T_MAX_MSB       = 0x24,
        T_MAX_LSB       = 0x25,
        CTRL_REG1       = 0x26,
        CTRL_REG2       = 0x27,
        CTRL_REG3       = 0x28,
        CTRL_REG4       = 0x29,
        CTRL_REG5       = 0x2A,
        OFF_P           = 0x2B,
        OFF_T           = 0x2C,
        OFF_H           = 0x2D
      };
      enum StatusBit: uint8_t {
        TDR = 0x02, // Bit 2
        PDR = 0x04  // Bit 3
      };
      enum CtrlReg1Bit: uint8_t {
        SBYB = 0x01, // Bit 1
        OST  = 0x02, // Bit 2
        RST  = 0x04, // Bit 3
        OS0  = 0x08, // Bit 4
        OS1  = 0x10, // Bit 5
        OS2  = 0x20, // Bit 6
        ALT  = 0x80  // Bit 7
      };
      enum PtDataCfgBit: uint8_t {
        TDEFE = 0x01, // Bit 1
        PDEFE = 0x02, // Bit 2
        DREM  = 0x04  // Bit 3
      };

   private:
      MPL3115A2& _self;

   public:
      std::shared_ptr<I2C> i2c;

      Impl(MPL3115A2& self, std::shared_ptr<I2C> i2c):
        _self(self),
        i2c(i2c)
      {
      }

      bool takeMeasurement(StatusBit statusBit) {
        // Ensure the device is in the ACTIVE power mode
        _self.setPowerMode(PowerMode::ACTIVE);

        // Clearing the OST bit forces a measurement to be taken
        i2c->setBit(Impl::SLAVE_ADDRESS, Register::CTRL_REG1, CtrlReg1Bit::OST, false);
  
        // Wait for the relevant status bit to indicate that the measurement data is available
        bool measurementAvailable = false;
        for (uint8_t i = 0; i < MEASUREMENT_ATTEMPTS; ++i) {
          uint8_t status = i2c->read(Impl::SLAVE_ADDRESS, Register::STATUS);

          if (status & statusBit) {
            measurementAvailable = true;
            break;
          }
        }

        return measurementAvailable;
      }

  }; // Class MPL3115A2::Impl


  MPL3115A2::MPL3115A2(std::shared_ptr<I2C> i2c):
    _pimpl{new Impl{*this, i2c}}
  {
    if (getDeviceId() != Impl::DEVICE_ID)
      throw std::runtime_error("Invalid MPL3115A2 pressure/altimeter device");
  }

  MPL3115A2::~MPL3115A2() {
  }

  uint8_t MPL3115A2::getDeviceId() {
    return _pimpl->i2c->read(Impl::SLAVE_ADDRESS, Impl::Register::WHO_AM_I);
  }

  void MPL3115A2::init() {
    //reset();
    setOversampleRatio();
    enableDataFlags();
    setMeasurementMode(MeasurementMode::ALTIMETER);
    // If interrupt mode set here
    setPowerMode(PowerMode::ACTIVE);
  }

  void MPL3115A2::reset() {
    _pimpl->i2c->setBit(Impl::SLAVE_ADDRESS, Impl::Register::CTRL_REG1, Impl::CtrlReg1Bit::RST);
  }


  void MPL3115A2::setPowerMode(PowerMode mode) {
    // Set the SBYB bit for ACTIVE mode, clear it for STANDBY mode
    _pimpl->i2c->setBit(Impl::SLAVE_ADDRESS, Impl::Register::CTRL_REG1, Impl::CtrlReg1Bit::SBYB, (mode == PowerMode::ACTIVE));
  }

  void MPL3115A2::setMeasurementMode(MeasurementMode mode) {
    // Set the ALT bit for altimeter mode, clear it for barometer mode
    _pimpl->i2c->setBit(Impl::SLAVE_ADDRESS, Impl::CtrlReg1Bit::ALT, (mode == MeasurementMode::ALTIMETER));
  }

  void MPL3115A2::enableDataFlags() {
    _pimpl->i2c->write(Impl::SLAVE_ADDRESS, Impl::Register::PT_DATA_CFG, (uint8_t) (Impl::PtDataCfgBit::DREM | Impl::PtDataCfgBit::PDEFE | Impl::PtDataCfgBit::TDEFE));
  }

  void MPL3115A2::setOversampleRatio(uint8_t osr) {
    // Must be in STANDBY mode to change settings??
    //setPowerMode(PowerMode::STANDBY);

    uint8_t os = log2(osr);
    _pimpl->i2c->setBits(Impl::SLAVE_ADDRESS, Impl::Register::CTRL_REG1, os << cntt0(Impl::CtrlReg1Bit::OS0), Impl::CtrlReg1Bit::OS0 | Impl::CtrlReg1Bit::OS1 | Impl::CtrlReg1Bit::OS2);  
  }

  /// Current pressure in Pa
  float MPL3115A2::getPressure() {
    if (!_pimpl->takeMeasurement(Impl::StatusBit::PDR))
      throw std::runtime_error("Sensor failed to generate the requested pressure reading");

    std::vector<uint8_t> result = _pimpl->i2c->read(Impl::SLAVE_ADDRESS, Impl::Register::OUT_P_MSB, (uint8_t) 3);

    /* Result is a left-shifted 20-bit number.
     * 18 bits whole with 2 bits fractional (decimal) in bits 4-5
     */
    long pressureWhole = (long) result[0] << 16 | (long) result[1] << 8 | (long) result[2];
    // Shift by 6 to remove bits 4-5 (the fractional part)
    pressureWhole >>= 6;

    long pressureFractional = (long) result[2] & 0b00110000;
    // Shift by 4 to move bits 4-5 back to zero
    pressureFractional >>= 4;
    // Convert 2 bits (4 possible values, equating to 0.25 decimal increments) into decimal
    float decimalPortion = (float) pressureFractional / 4.0;

    return (float) pressureWhole + decimalPortion;
  }

  float MPL3115A2::getAltitude() {
    if (!_pimpl->takeMeasurement(Impl::StatusBit::PDR))
      throw std::runtime_error("Sensor failed to generate the requested pressure reading");

    std::vector<uint8_t> result = _pimpl->i2c->read(Impl::SLAVE_ADDRESS, Impl::Register::OUT_P_MSB, (uint8_t) 3);

    /* The lower 4 bits of the LSB contain the decimal portion
     * (These 4 bits equate to 16 possible decimal values or 0.0625 increments).
     */
    float decimalPortion = (result[2] >> 4) / 16.0;

    return (float) (result[0] << 8 | result[1]) + decimalPortion;
  }

  float MPL3115A2::getTemperature() {
    if (!_pimpl->takeMeasurement(Impl::StatusBit::TDR))
      throw std::runtime_error("Sensor failed to generate the requested temperature reading");

    // TODO: complete
    return 0.0;
  }

} // Namespace Octo

