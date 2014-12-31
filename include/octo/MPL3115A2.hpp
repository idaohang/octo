#ifndef OCTO_MPL3115A2_HPP
#define OCTO_MPL3115A2_HPP

#include "I2C.hpp"
#include <cstdint>   // Standard integer types (eg. uint8_t)
#include <memory>    // std::shared_ptr


namespace Octo {

  class MPL3115A2 { 
    public:
      // Configuration options
      static const uint8_t OVERSAMPLE_RATIO = 128;

      // Internal
      enum class PowerMode: bool {
        STANDBY,
        ACTIVE
      };
      enum class MeasurementMode: bool {
        BAROMETER,
        ALTIMETER
      };


    private:
      // Pimpl idiom
      class Impl;
      std::unique_ptr<Impl> _pimpl;


    public:
      MPL3115A2(std::shared_ptr<I2C> i2c);
      ~MPL3115A2();

      uint8_t getDeviceId();
      void init();
      void reset();
      void setPowerMode(PowerMode mode);
      void setMeasurementMode(MeasurementMode mode);
      void enableDataFlags();
      void setOversampleRatio(uint8_t osr = OVERSAMPLE_RATIO);
      float getPressure();
      float getAltitude();
      float getTemperature();

  }; // Class MPL3115A2

} // Namespace Octo

#endif // OCTO_MPL3115A2_HPP

