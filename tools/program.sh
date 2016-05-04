#! /bin/sh

read  -p "Restart ESP8266 in program mode and Press any key to continue... " ANSWER

# Program ESP firmware
cd /opt/Espressif/ESP8266_SDK/at_v0.20_on_SDKv0.9.3/
./build_and_program.sh

# Program bootloader
/opt/arduino/hardware/tools/avrdude -C/opt/arduino/hardware/tools/avrdude.conf -cusbasp -pm328p -v -Pusb -V -U efuse:w:0x07:m -U hfuse:w:0xDA:m -U lfuse:w:0xE2:m -U flash:w:/opt/arduino/hardware/arduino/bootloaders/atmega/ATmegaBOOT_168_rodi.hex:i

# Ask user to reboot and start ESP in program mode
# Ask user to connect to WiFi Network
#read -p "Solder bridge, connect to ESP8266 AP and Press any key to continue... " ANSWER

# Program arduino sample code
#cd /home/gary/data/projects/gary/git/rodi-dev/rodi-code/programmer
#./program_rodi.py /tmp/build312397392944530524.tmp/Blink.cpp.hex
