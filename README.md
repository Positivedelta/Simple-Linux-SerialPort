# A Simple C++ SerialPort Class for Linux

#### Prerequisites
- Linux
- CMake 3.16 (can be tweaked to use older versions)
- C++ 2017 (older versions are fine)

#### Build Instructions
To build and run the test application, use:

```
mkdir build
cd build
cmake ..
make -j4
./test-serial-port
```

#### Install Instructions
To install the core shared library, the include files and test applications, use:

```
make install
```

The following default paths are used, edit `CMakeLists.txt` if you want to update these
- The shared library is installed to `bit-parallel/lib`
- The include files are installed to `bit-parallel/include/communications/serial`
- The test appliactions are installed to `bit-parallel/bin/serial`

#### Notes
- Tested on a Raspberry Pi 4 running the official 32- or 64-bit OS
- Make sure that the chosen device is available, i.e. not designated as a console
- Please contact me at max.vandaalen@bitparallel.com if you have any questions
