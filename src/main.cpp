#include <octo/LinuxI2C.hpp>
#include <octo/MPL3115A2.hpp>
//#include "MPU9150.hpp"
//#include "Si4463.hpp"

#include <iostream> // printf
#include <memory>   // Shared pointers


// Global constants
const uint8_t I2C_BUS = 2;
//const uint8_t SPI_BUS = 1;


using namespace Octo;

int main(int argc, char* argv[]) {
  try {
    // Communication busses
    auto i2c = std::make_shared<LinuxI2C>(I2C_BUS);
    //auto spi = std::make_shared<LinuxSPI>(SPI_BUS);

    // Devices
    MPL3115A2 altimeter(i2c);
    //MPU9150   imu(i2c);
    //Si4463    rf (spi);

    altimeter.init();
    //printf("Pressure:    %.2f Pa\n", altimeter.getPressure());
    printf("Altitude:    %.4f m\n", altimeter.getAltitude());
    //printf("Temperature: %f\n", altimeter.getTemperature());
  } catch (std::exception& e) {
    printf("Error: %s\n", e.what());
  }

  return 0;
}

