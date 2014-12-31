#ifndef OCTO_LINUXI2C_HPP
#define OCTO_LINUXI2C_HPP

#include "I2C.hpp"


namespace Octo {

  class LinuxI2C: public I2C {
    private:
      // Pimpl idiom
      class Impl;
      std::unique_ptr<Impl> _pimpl;

    protected:
      void _sendTransaction(uint8_t slaveAddress, std::vector<Message>& transaction);

    public:
      LinuxI2C(uint8_t bus);
      ~LinuxI2C();

      void open();   
      void close();

  }; // Class LinuxI2C

} // Namespace Octo

#endif // OCTO_LINUXI2C_HPP

