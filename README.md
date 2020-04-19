# TWIPPY (Two Wheeled Inverted Pendulum PY) low-level firmware

## Description
Two wheel inverted pendulum robot project. 
This is the firmware of the STM32F446RE microcontroller of the Nucleo Board used to control the robot.

## Dependencies

### Toolchain
```bash
sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
sudo apt-get update
sudo apt install gcc-arm-embedded
```

## Compilation and installation

## Build
```bash
cd twippy_firmware
mkdir bin build
make
```

## Flash with dfu
To flash after the bootloader at address 0x08004000: <br />
```bash
sudo dfu-util -d 0483:df11 -a 0 -s 0x08004000 -D bin/twippy_firmware.bin
```

To flash at the beginning of the flash (address 0x08000000): <br />
```bash
sudo dfu-util -d 0483:df11 -a 0 -s 0x08000000 -D bin/twippy_firmware.bin
```
To flash with a .dfu file: <br />
```bash
sudo dfu-util -d 0483:df11 -a 0 -D bin/twippy_firmware.bin
```

## Flash using stlink
```bash
git clone https://github.com/texane/stlink.git
cd stlink
make release
cd build/Release/
./st-flash write ../bin/twippy_firmware.bin 0x8000000
st-flash write bin/twippy_firmware.bin 0x8000000
```
## Utils

### To access serial port
picocom /dev/ttyUSB0 -b 115200
