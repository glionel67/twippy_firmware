If the Crazyflie 2.0 firmware was compiled with CLOAD=1 (default option) the binary should be flashed after the bootloader at address 0x08004000:
sudo dfu-util -d 0483:df11 -a 0 -s 0x08004000 -D cf2.bin

If the Crazyflie 2.0 firmware was compiled with CLOAD=0 the binary should be flashed in the beginning of the flash (address 0x08000000). WARNING, this will overwrite the radio-bootloader if it is there. You can however flash the bootloader back this same way with DFU:
sudo dfu-util -d 0483:df11 -a 0 -s 0x08000000 -D cf2.bin

To flash with a .dfu file:
sudo dfu-util -d 0483:df11 -a 0 -D cf2.dfu

################################################################################
Install https://github.com/texane/stlink.git
cd stlink
make release
cd build/Release/
./st-flash write ~/Workspace/stm32f446/bin/stm32f446.bin 0x8000000
################################################################################


Help:
https://github.com/starnight/STM32F4/tree/master/FreeRTOS-STM32F4
https://github.com/wangyeee/STM32F4-FreeRTOS/blob/master/main.c

