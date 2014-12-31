#include <octo/LinuxI2C.hpp>
#include <fcntl.h>         // open, read, write
#include <sys/ioctl.h>     // ioctl
#include <linux/i2c.h>     // i2c_msg 
#include <linux/i2c-dev.h> // i2c_rdwr_ioctl_data, I2C_RDWR, I2C_FUNCS
#include <stdexcept>       // std::runtime_error
#include <cassert>         // assert


namespace Octo {

  // Pimpl idiom class
  class LinuxI2C::Impl {
    public:
      int     file;
      uint8_t bus;

      Impl(uint8_t bus):
        bus(bus)
      {
      }
  }; // Class LinuxI2C::Impl


  void LinuxI2C::_sendTransaction(uint8_t slaveAddress, std::vector<Message>& transaction) {
    int numMessages = transaction.size();
    struct i2c_rdwr_ioctl_data request;
    struct i2c_msg linuxTransaction[numMessages];
    request.nmsgs = numMessages;
    request.msgs  = linuxTransaction;

    // Debug
    printf("I2C [Bus %d; Slave 0x%02x]: ", _pimpl->bus, slaveAddress);
    if (numMessages > 1)
      printf("Multipart\n");

    std::vector<size_t> reads;
    for (uint8_t i = 0; i < numMessages; ++i) {
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
    if (ioctl(_pimpl->file, I2C_RDWR, &request) < 0)
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

  LinuxI2C::LinuxI2C(uint8_t bus):
    _pimpl{new Impl{bus}}
  {
    open();
   }

  LinuxI2C::~LinuxI2C() {
    close();
  }

  void LinuxI2C::open() {
     assert(!_pimpl->file);

    char device[32];
    sprintf(device, "/dev/i2c-%d", _pimpl->bus);

    if ((_pimpl->file = ::open(device, O_RDWR)) < 0)
      throw std::runtime_error("Could not open POSIX I2C device");

    unsigned long funcs;
    ioctl(_pimpl->file, I2C_FUNCS, &funcs);
    if (!(funcs & I2C_FUNC_I2C))
      throw std::runtime_error("Required POSIX I2C functionality not supported");
  }

  void LinuxI2C:: close() {
     assert(_pimpl->file);

    ::close(_pimpl->file);
    _pimpl->file = 0;
  }

} // Namespace Octo

