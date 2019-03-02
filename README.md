# TWIPPY (Two Wheeled Inverted Pendulum PY) low level firmware

# Build
cd twippy_firmware <br />
mkdir bin build <br />
make

# Flash with dfu
To flash after the bootloader at address 0x08004000: <br />
sudo dfu-util -d 0483:df11 -a 0 -s 0x08004000 -D bin/twippy_firmware.bin <br />

To flash at the beginning of the flash (address 0x08000000): <br />
sudo dfu-util -d 0483:df11 -a 0 -s 0x08000000 -D bin/twippy_firmware.bin <br />

To flash with a .dfu file: <br />
sudo dfu-util -d 0483:df11 -a 0 -D bin/twippy_firmware.bin <br />

# To flash using stlink
git clone https://github.com/texane/stlink.git <br />
cd stlink <br />
make release <br />
cd build/Release/ <br />
./st-flash write ../bin/twippy_firmware.bin 0x8000000 <br />
st-flash write bin/twippy_firmware.bin 0x8000000 <br />
