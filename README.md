# sts3215_control

C++11 driver for Feetech STS3215 serial bus servos. Windows serial backend; Arduino backend not written yet. Work in progress.

## Build

```
g++ -std=c++11 -Isrc -o build/driver_test.exe tools/windows/driver_test.cpp src/driver.cpp src/packet.cpp src/convert.cpp src/register.cpp src/driver_errors.cpp src/servo_errors.cpp src/platforms/windows.cpp
```

## Run

```
build\driver_test.exe        offline packet checks, no hardware
build\driver_test.exe COM11 1 ping and read servo ID 1 on COM11 (1 Mbaud)
```

Offline driver test with a simulated bus:

```
g++ -std=c++11 -Isrc -o build/simulated.exe tools/windows/simulated.cpp src/driver.cpp src/packet.cpp src/convert.cpp src/register.cpp src/driver_errors.cpp src/servo_errors.cpp
build\simulated.exe
```

## Layout

```
tools/arduino/        Arduino ping test (LED on = servo answered); needs the library installed
src/                  driver library, compiled by both g++ and the Arduino IDE
tools/windows/        Windows-only programs (driver_test, simulated, find_port.bat)
notes/                scservo_sdk (vendor Python SDK) exploration that preceded this driver
build/                desktop build output, not tracked
```
