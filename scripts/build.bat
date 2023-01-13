@ECHO off

mkdir build
cd build

H:\SDKs\nRF\toolchains\v2.2.0\opt\bin\cmake -GNinja ..
H:\SDKs\nRF\toolchains\v2.2.0\opt\bin\ninja.exe

copy zephyr\zephyr.hex ..\remote_id.hex

