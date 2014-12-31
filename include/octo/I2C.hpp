#ifndef OCTO_I2C_HPP
#define OCTO_I2C_HPP

#include <cstdint> // Standard integer types (eg. uint8_t)
#include <vector>  // std::vector
#include <memory>  // std::unique_ptr


namespace Octo {

  class I2C {
    private:
      // Pimpl idiom
      class Impl;
      std::unique_ptr<Impl> _pimpl;

    protected:
      struct Message {
        enum class Type: bool {
          READ,
          WRITE
        } type;
        std::vector<uint8_t> data;
      };

      // Implementation/platform-specific I2C write method (pure virtual)
      virtual void _sendTransaction(uint8_t slaveAddress, std::vector<Message>& messages) = 0;

    public:
      I2C();
      ~I2C();

      // -- Read methods --
      // Read word/block data
      std::vector<uint8_t>& read(uint8_t slaveAddress, uint8_t registerAddress, uint8_t numBytes, bool endOfTransaction = true);

      // Read byte data
      uint8_t& read(uint8_t slaveAddress, uint8_t registerAddress, bool endOfTransaction = true);

      // Read byte
      uint8_t& read(uint8_t slaveAddress);

      // -- Write methods --
      // Write block data
      void write(uint8_t slaveAddress, uint8_t registerAddress, const std::vector<uint8_t> data, bool endOfTransaction = true);

      // Write byte data
      void write(uint8_t slaveAddress, uint8_t registerAddress, bool endOfTransaction = true);

      // Write byte
      void write(uint8_t slaveAddress, uint8_t registerAddress, uint8_t data, bool endOfTransaction = true);

      // -- Other utility methods --
      // Combined read/write to set/clear one or more bits of a register
      void setBits(uint8_t slaveAddress, uint8_t registerAddress, uint8_t value, uint8_t mask);

      // Combined read/write to set/clear a single bit of a register (wraps method above)
      void setBit(uint8_t slaveAddress, uint8_t registerAddress, uint8_t value, bool set = true);

  }; // Class I2C

} // Namespace Octo

#endif // OCTO_I2C_HPP

