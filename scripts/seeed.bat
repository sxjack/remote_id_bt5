@ECHO off

set PORT=COM23
REM set PORT=COM34

REM TO DO
REM Find out what Arduino does to put the board into bootloader over the serial link.
REM

set PYTHON=H:\SDKs\nRF\toolchains\v2.2.0\opt\bin\python.exe
set UF2CONV=H:\SDKs\nRF\v2.2.0\zephyr\scripts\build\uf2conv.py
set NRFUTIL="C:\Users\xxxxx\AppData\Local\Arduino15\packages\Seeeduino\hardware\nrf52\1.0.0\tools\adafruit-nrfutil\win32\adafruit-nrfutil.exe"

%PYTHON% %UF2CONV% -f 0x1b57745f -o remote_id.uf2 remote_id.hex
%NRFUTIL% dfu genpkg --dev-type 0x0052 --application remote_id.hex dfu-package.zip
%NRFUTIL% dfu serial --package dfu-package.zip -p %PORT% -b 115200 --singlebank
