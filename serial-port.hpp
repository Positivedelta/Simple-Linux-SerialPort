//
// (c) Bit Parallel Ltd (Max van Daalen), March 2022
//

#ifndef H_SERIAL_PORT
#define H_SERIAL_PORT

#include <cstdint>
#include <thread>
#include <string>

#include "read-listener.hpp"

class SerialPort
{
    public:
        static inline const uint8_t NEW_LINE[] = {0x0d, 0x0a};

    private:
        const static inline int32_t RX_BUFFER_SIZE = 4096;
        const static inline int32_t RX_SELECT_TIMEOUT_US = 100000;
        const static inline ReadListener DEFAULT_RX_LISTENER = [](const uint8_t rxedBytes[], const int32_t length) {};

        const bool enableReceiver;
        const std::string& deviceName;
        int32_t deviceFd;
        bool doReceive;
        std::thread rxTask;

    public:
        SerialPort(const std::string& deviceName, const int32_t baudRate, const bool enableReceiver = false, const ReadListener rxListener = DEFAULT_RX_LISTENER);
        void write(const uint8_t bytes[], int32_t length) const;
        void write(const uint8_t singleByte) const;
        void print(const std::string& text) const;
        void printLine() const;
        void printLine(const std::string& text) const;
        ~SerialPort();
};

#endif
