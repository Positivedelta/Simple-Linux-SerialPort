//
// (c) Bit Parallel Ltd (Max van Daalen), March 2022
//

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "serial-port.hpp"

SerialPort::SerialPort(const std::string& deviceName, const int32_t baudRate, const bool enableReceiver, const ReadListener rxListener):
    deviceName(deviceName), enableReceiver(enableReceiver) {

        // these baud rates are limited to the predefined magic numbers set by the Open Group!
        // only the commonly used speeds are defined below, others exist
        //
        int32_t termiosBaudRate = 0;
        switch (baudRate)
        {
            // note, there are lower values that can be added in if needed
            //
            case 1200:
                termiosBaudRate = B1200;
                break;

            case 2400:
                termiosBaudRate = B2400;
                break;

            case 4800:
                termiosBaudRate = B4800;
                break;

            case 9600:
                termiosBaudRate = B9600;
                break;

            case 57600:
                termiosBaudRate = B57600;
                break;

            case 115200:
                termiosBaudRate = B115200;
                break;

            case 230400:
                termiosBaudRate = B230400;
                break;

            case 460800:
                termiosBaudRate = B460800;
                break;

            case 921600:
                termiosBaudRate = B921600;
                break;

            default:
            {
                throw std::string("Invalid baud rate for ") + deviceName + std::string(", supported values are 1200, 2400, 4800, 9600, 57600, 115200, 230400, 460800 and 921600");
            }
        }

        deviceFd = open(deviceName.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (!deviceFd) throw std::string("Unable to open the serial port on device: ") + deviceName;

        // set the tx/rx baud rate
        //
        struct termios options;
        tcgetattr(deviceFd, &options);
        cfsetspeed(&options, termiosBaudRate);
        options.c_iflag = IGNPAR;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag |= (CS8 & CLOCAL & CREAD);
        cfmakeraw(&options);
        tcsetattr(deviceFd, TCSANOW, &options);

        // if enabled, start up the receiver task
        //
        doReceive = true;
        if (enableReceiver)
        {
            rxTask = std::thread([this, rxListener]() {
                struct timeval timeout;
                timeout.tv_sec = 0;
                timeout.tv_usec = RX_SELECT_TIMEOUT_US;

                fd_set readFdSet;
                uint8_t rxBuffer[RX_BUFFER_SIZE];
                const int32_t maxFd = 1 + deviceFd;
                while (doReceive)
                {
                    FD_ZERO(&readFdSet);
                    FD_SET(deviceFd, &readFdSet);

                    auto fdCount = select(maxFd, &readFdSet, nullptr, nullptr, &timeout);
                    if (fdCount > 0 && FD_ISSET(deviceFd, &readFdSet))
                    {
                        // FIXME! currently ignoring read() errors, i.e. a -ve return value
                        //        perhaps add an error callback...
                        //
                        const int32_t bytesRead = ::read(deviceFd, rxBuffer, RX_BUFFER_SIZE);
                        if (bytesRead > 0) rxListener(rxBuffer, bytesRead);
                    }
                }
            });
        }
}

void SerialPort::write(const uint8_t bytes[], int32_t length) const
{
    int32_t i = 0;
    int32_t writeStatus; 
    while ((length > 0) && (writeStatus = ::write(deviceFd, &bytes[i], length)) != length)
    {
        if (writeStatus < 0)
        {
            if ((errno == EINTR) || (errno == EAGAIN) || (errno == EWOULDBLOCK)) continue;

            throw std::string("Serial port write error on device: ") + deviceName + std::string(", Status: ") + std::to_string(writeStatus);
        }

        length -= writeStatus;
        i += writeStatus;
    }
}

void SerialPort::write(const uint8_t singleByte) const
{
    const uint8_t bytes[1] = {singleByte};
    write(bytes, 1);
}

void SerialPort::print(const std::string& text) const
{
    write(reinterpret_cast<const uint8_t*>(text.data()), text.size());
}

void SerialPort::printLine() const
{
    write(NEW_LINE, sizeof(NEW_LINE));
}

void SerialPort::printLine(const std::string& text) const
{
    print(text);
    write(NEW_LINE, sizeof(NEW_LINE));
}

SerialPort::~SerialPort()
{
    if (enableReceiver)
    {
        doReceive = false;
        rxTask.join();
    }

    close(deviceFd);
}
