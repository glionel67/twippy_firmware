# TWIPPY (Two Wheeled Inverted Pendulum PY) low level firmware

# Build
cd twippy_firmware
make

# Flash with dfu
To flash after the bootloader at address 0x08004000:
sudo dfu-util -d 0483:df11 -a 0 -s 0x08004000 -D bin/twippy_firmware.bin

To flash at the beginning of the flash (address 0x08000000):
sudo dfu-util -d 0483:df11 -a 0 -s 0x08000000 -D bin/twippy_firmware.bin

To flash with a .dfu file:
sudo dfu-util -d 0483:df11 -a 0 -D bin/twippy_firmware.bin

# To flash using stlink
git clone https://github.com/texane/stlink.git
cd stlink
make release
cd build/Release/
./st-flash write ../bin/twippy_firmware.bin 0x8000000

