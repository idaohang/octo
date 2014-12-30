#ifndef OCTO_LINUXI2C_HPP
#define OCTO_LINUXI2C_HPP

#include "I2C.hpp"
#include <fcntl.h>         // open, read, write
#include <sys/ioctl.h>     // ioctl
#include <linux/i2c.h>     // i2c_msg 
#include <linux/i2c-dev.h> // i2c_rdwr_ioctl_data, I2C_RDWR, I2C_FUNCS
#include <stdexcept>       // std::runtime_error
#include <cassert>         // assert


namespace Octo {

  class LinuxI2C: public I2C {
    protected:
      // Constants
      static const uint8_t READ_TRIES = 5;

      int     _file;
      uint8_t _bus;
      uint8_t _slaveAddress;


      void _sendTransaction(uint8_t slaveAddress, std::vector<Message>& transaction) {
        int numMessages = transaction.size();
        struct i2c_rdwr_ioctl_data request;
        struct i2c_msg linuxTransaction[numMessages];
        request.nmsgs = numMessages;
        request.msgs  = linuxTransaction;

        // Debug
        printf("I2C [Bus %d; Slave 0x%02x]: ", _bus, slaveAddress);
        if (numMessages > 1)
          printf("Multipart\n");

        std::vector<size_t> reads;
        for (size_t i = 0; i < numMessages; ++i) {
          Message& message      = transaction[i];
          i2c_msg& linuxMessage = linuxTransaction[i];
          bool isRead = (message.type == Message::Type::READ);

          linuxMessage.addr  = slaveAddress;
          linuxMessage.flags = (isRead ? I2C_M_RD : 0);
          linuxMessage.len   = message.data.size();
          linuxMessage.buf   = message.data.data();

          // If this is a READ, add it to our read vector so we can update the vector size after the read
          if (isRead)
            reads.push_back(i);

          // Debug message
          if (numMessages > 1)
            printf("  [Part %d]: ", i + 1);
          if (isRead) {
            printf("Read ");
            if (numMessages > 1)
              printf(" %3dB\n", linuxMessage.len);
            else
              printf("%dB\n", linuxMessage.len);
          } else {
            printf("Write");
            for (int j = 0; j < linuxMessage.len; ++j)
              printf(" 0x%02x", linuxMessage.buf[j]);
            printf("\n");
          }
        }

        // Send the messages to the kernel-mode I2C driver
        if (ioctl(_file, I2C_RDWR, &request) < 0)
          throw std::runtime_error("Could not communicate with POSIX I2C slave");

        // Resize READ message response vectors
        for (const size_t& i: reads) {
          Message& message      = transaction[i];
          i2c_msg& linuxMessage = linuxTransaction[i];
          message.data.resize(linuxMessage.len);

          // Debug message
          if (numMessages > 1)
            printf("  ");
          printf("  - Received");
          for (int j = 0; j < linuxMessage.len; ++j)
            printf(" 0x%02x", linuxMessage.buf[j]);
          if (numMessages > 1)
            printf(" to Part %d", i + 1);
          printf("\n");
        }
        printf("\n");
      }


    public:
      LinuxI2C(uint8_t bus):
        _bus(bus)
      {
        open();
      }

      ~LinuxI2C() {
        close();
      }

      void open() {
        assert(!_file);

        char device[32];
        sprintf(device, "/dev/i2c-%d", _bus);

        if ((_file = ::open(device, O_RDWR)) < 0)
          throw std::runtime_error("Could not open POSIX I2C device");

        unsigned long funcs;
        ioctl(_file, I2C_FUNCS, &funcs);
        if (!(funcs & I2C_FUNC_I2C))
          throw std::runtime_error("Required POSIX I2C functionality not supported");
      }
    
      void close() {
        assert(_file);
      
        ::close(_file);
        _file = 0;
      }

  }; // Class LinuxI2C

} // Namespace Octo

#endif // OCTO_LINUXI2C_HPP

