//
// (c) Bit Parallel Ltd, August 2023
//

#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

#include "serial_port.hpp"

int32_t main()
{
    try
    {
        // create and open the serial port, enable the receiver thread add a listener for any RXed data
        // note, the destructor will close the selected device
        //
        auto serialPort = bpl::SerialPort("/dev/serial0", 115200, true, [](const uint8_t bytes[], const int32_t length) {
            const auto asText = std::string(reinterpret_cast<const char*>(bytes), length);
            std::cout << asText << std::flush;
        });

        // say hello and then sleep for 10 seconds to allow some data to be RXed...
        //
        serialPort.printLine("Hello World!");
        serialPort.print("Please type some text: ");

        std::this_thread::sleep_for(std::chrono::seconds(20));
        serialPort.printLine();
    }
    catch (const std::string& message)
    {
        std::cout << "Exception, " + message << std::endl;
    }

    return 0;
}
