# sts3215_control

C++11 driver for Feetech STS3215 serial bus servos. Windows serial backend; Arduino backend not written yet. Work in progress.

## Build

```
g++ -std=c++11 -o check.exe main.cpp driver.cpp packet.cpp convert.cpp register.cpp platforms/windows.cpp
```

## Run

```
check.exe              offline packet checks, no hardware
check.exe COM11 1      ping and read servo ID 1 on COM11 (1 Mbaud)
```

Offline driver test with a simulated bus:

```
g++ -std=c++11 -o test_driver.exe test_driver.cpp driver.cpp packet.cpp convert.cpp register.cpp
test_driver.exe
```
