#include <octo/I2C.hpp>
#include <map>          // std::map


namespace Octo {

  // Pimpl idiom class
  class I2C::Impl {
    private:
      I2C& _self;

      std::map<uint8_t, std::pair<std::vector<Message>, bool>> _transactions; 

      inline void _handleCompletion(std::vector<Message>& transaction, bool& complete) {
        if (complete) {
          transaction.clear();
          complete = false;
        }
      }

    public:
      Impl(I2C& self):
        _self(self)
      {
      }

      std::vector<uint8_t>& _read(uint8_t slaveAddress, uint8_t numBytes, bool endOfTransaction = true) {
        std::pair<std::vector<Message>, bool>& transactionPair = _transactions[slaveAddress];
        std::vector<Message>& transaction = transactionPair.first;
        bool& complete = transactionPair.second;

        // Clear the transaction if we're onto a new one
        _handleCompletion(transaction, complete);

        // Add the Message to the transaction and give it enough memory to store the result
        transaction.push_back({
          Message::Type::READ
        });
        std::vector<uint8_t>& result = transaction.back().data;
        result.resize(numBytes);

        // Send the transaction if we're ready
        if (endOfTransaction) {
          _self._sendTransaction(slaveAddress, transaction);

          complete = true;
        }

        return result;
      }

      void _write(uint8_t slaveAddress, const std::vector<uint8_t>& data, bool endOfTransaction = true) {
        std::pair<std::vector<Message>, bool>& transactionPair = _transactions[slaveAddress];
        std::vector<Message>& transaction = transactionPair.first;
        bool& complete = transactionPair.second;

        // Clear the transaction if we're onto a new one
        _handleCompletion(transaction, complete);

        // Add the Message to the transaction
        transaction.push_back({
          Message::Type::WRITE,
          data
        });

        // Send the transaction if we're ready
        if (endOfTransaction) {
          _self._sendTransaction(slaveAddress, transaction);

          complete = true;
        }
      }

  }; // Class I2C::Impl


  // Constructor, destructor
  I2C::I2C():
    _pimpl{new Impl{*this}}
  {
  }

  I2C::~I2C() {
  }

  // -- Read methods --
  // Read word/block data
  std::vector<uint8_t>& I2C::read(uint8_t slaveAddress, uint8_t registerAddress, uint8_t numBytes, bool endOfTransaction) {
    write(slaveAddress, registerAddress, false);
    return _pimpl->_read(slaveAddress, numBytes, endOfTransaction);
  }

  // Read byte data
  uint8_t& I2C::read(uint8_t slaveAddress, uint8_t registerAddress, bool endOfTransaction) {
    return (read(slaveAddress, registerAddress, 1, endOfTransaction))[0];
  }

  // Read byte
  uint8_t& I2C::read(uint8_t slaveAddress) {
    return (_pimpl->_read(slaveAddress, 1))[0];
  }

  // -- Write methods --
  // Write block data
  void I2C::write(uint8_t slaveAddress, uint8_t registerAddress, const std::vector<uint8_t> data, bool endOfTransaction) {
    std::vector<uint8_t> request = {registerAddress};
    request.insert(request.end(), data.begin(), data.end());
    _pimpl->_write(slaveAddress, request, endOfTransaction);
  }

  // Write byte data
  void I2C::write(uint8_t slaveAddress, uint8_t registerAddress, bool endOfTransaction) {
    _pimpl->_write(slaveAddress, std::vector<uint8_t>{registerAddress}, endOfTransaction);
  }

  // Write byte
  void I2C::write(uint8_t slaveAddress, uint8_t registerAddress, uint8_t data, bool endOfTransaction) {
    _pimpl->_write(slaveAddress, {registerAddress, data}, endOfTransaction);
  }

  // -- Other utility methods --
  void I2C::setBits(uint8_t slaveAddress, uint8_t registerAddress, uint8_t value, uint8_t mask) {
    // Get current register value
    uint8_t currentValue = read(slaveAddress, registerAddress);

    // Get the new value
    uint8_t newValue = (currentValue ^ mask) | value;

    // Write the new value if it has changed
    if (currentValue != newValue)
      write(slaveAddress, registerAddress, newValue);
  }

  void I2C::setBit(uint8_t slaveAddress, uint8_t registerAddress, uint8_t value, bool set) {
    if (set)
      setBits(slaveAddress, registerAddress, value, value);
    else
      setBits(slaveAddress, registerAddress, 0x00, value);
  }

} // Namespace Octo

