@echo off
rem finds the servo adapter's COM port: lists the ports with the adapter
rem unplugged, again with it plugged in, and prints whichever one is new
setlocal
set "BEFORE=%TEMP%\sts_ports_before.txt"
set "AFTER=%TEMP%\sts_ports_after.txt"

echo UNPLUG the servo adapter, then press a key.
pause >nul
powershell -NoProfile -Command "[System.IO.Ports.SerialPort]::GetPortNames()" > "%BEFORE%"
rem findstr misbehaves on an empty pattern file
echo NONE>> "%BEFORE%"

echo PLUG IN the servo adapter, then press a key.
pause >nul
rem give windows a moment to create the port
timeout /t 3 /nobreak >nul 2>&1
powershell -NoProfile -Command "[System.IO.Ports.SerialPort]::GetPortNames()" > "%AFTER%"

set "FOUND="
for /f %%p in ('findstr /v /x /l /g:"%BEFORE%" "%AFTER%"') do (
  echo Adapter is on %%p      test with: check.exe %%p 1
  set "FOUND=1"
)
if not defined FOUND (
  echo No new port appeared. Check the cable, or install the adapter's driver
  echo ^(look for an unknown device in Device Manager^).
)
del "%BEFORE%" "%AFTER%" 2>nul
endlocal
