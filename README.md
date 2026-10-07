# sts3215_control

C++11 driver for Feetech STS3215 serial bus servos. Windows serial backend; Arduino backend not written yet. Work in progress.

## Build

```
g++ -std=c++11 -Isrc -o build/check.exe tools/windows/check.cpp src/driver.cpp src/packet.cpp src/convert.cpp src/register.cpp src/platforms/windows.cpp
```

## Run

```
build\check.exe        offline packet checks, no hardware
build\check.exe COM11 1 ping and read servo ID 1 on COM11 (1 Mbaud)
```

Offline driver test with a simulated bus:

```
g++ -std=c++11 -Isrc -o build/test_driver.exe tools/windows/test_driver.cpp src/driver.cpp src/packet.cpp src/convert.cpp src/register.cpp
build\test_driver.exe
```

## Layout

```
sts3215_control.ino   Arduino sketch entry (folder name must match)
src/                  driver library, compiled by both g++ and the Arduino IDE
tools/windows/        Windows-only programs (check, test_driver, find_port.bat)
notes/                scservo_sdk (vendor Python SDK) exploration that preceded this driver
build/                desktop build output, not tracked
```
