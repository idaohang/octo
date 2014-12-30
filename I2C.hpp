#ifndef OCTO_I2C_HPP
#define OCTO_I2C_HPP

#include <cstdint> // Standard integer types (eg. uint8_t)
#include <vector>  // std::vector
#include <map>     // std::map


namespace Octo {

  class I2C {
    protected:
      struct Message {
        enum class Type: bool {
          READ,
          WRITE
        } type;
        std::vector<uint8_t> data;
      };

      std::map<uint8_t, std::pair<std::vector<Message>, bool>> _transactions; 

      // Implementation/platform-specific I2C write method (pure virtual)
      virtual void _sendTransaction(uint8_t slaveAddress, std::vector<Message>& messages) = 0;

      inline void _handleCompletion(std::vector<Message>& transaction, bool& complete) {
        if (complete) {
          transaction.clear();
          complete = false;
        }
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
          _sendTransaction(slaveAddress, transaction);

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
          _sendTransaction(slaveAddress, transaction);

          complete = true;
        }
      }


    public:
      I2C() {
      }

      // -- Read methods --
      // Read word/block data
      std::vector<uint8_t>& read(uint8_t slaveAddress, uint8_t registerAddress, uint8_t numBytes, bool endOfTransaction = true) {
        write(slaveAddress, registerAddress, false);
        return _read(slaveAddress, numBytes, endOfTransaction);
      }

      // Read byte data
      uint8_t& read(uint8_t slaveAddress, uint8_t registerAddress, bool endOfTransaction = true) {
        return (read(slaveAddress, registerAddress, 1, endOfTransaction))[0];
      }

      // Read byte
      uint8_t& read(uint8_t slaveAddress) {
        return (_read(slaveAddress, 1))[0];
      }

      // -- Write methods --
      // Write block data
      void write(uint8_t slaveAddress, uint8_t registerAddress, const std::vector<uint8_t> data, bool endOfTransaction = true) {
        std::vector<uint8_t> request = {registerAddress};
        request.insert(request.end(), data.begin(), data.end());
        _write(slaveAddress, request, endOfTransaction);
      }

      // Write byte data
      void write(uint8_t slaveAddress, uint8_t registerAddress, bool endOfTransaction = true) {
        _write(slaveAddress, std::vector<uint8_t>{registerAddress}, endOfTransaction);
      }

      // Write byte
      void write(uint8_t slaveAddress, uint8_t registerAddress, uint8_t data, bool endOfTransaction = true) {
        _write(slaveAddress, {registerAddress, data}, endOfTransaction);
      }

      // -- Other utility methods --
      void setBits(uint8_t slaveAddress, uint8_t registerAddress, uint8_t value, uint8_t mask) {
        // Get current register value
        uint8_t currentValue = read(slaveAddress, registerAddress);

        // Get the new value
        uint8_t newValue = currentValue ^ mask | value;

        // Write the new value if it has changed
        if (currentValue != newValue)
          write(slaveAddress, registerAddress, newValue);
      }

      void setBit(uint8_t slaveAddress, uint8_t registerAddress, uint8_t value, bool set = true) {
        if (set)
          setBits(slaveAddress, registerAddress, value, value);
        else
          setBits(slaveAddress, registerAddress, 0x00, value);
      }

  }; // Class I2C

} // Namespace Octo

#endif // OCTO_I2C_HPP

