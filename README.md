# rpi-chronosrf

Raspberry PI CC1101 Interface code, Right now this is able to reliably sniff 902MHz GFSK modulation, changing the smart rf reg values should allow this to work for other modulations as well. 

Wiring CC1101 to Raspberry pi:

GD0 connected to GPIO24
GD2 Connected to GPIO23
MISO and GPIO25 are shorted to sense CC1101 CHIP Ready i.e state of MISO on GPIO 25
CC1101's Chip Select is connected to GPIO22

MISO, MOSI, SCK are connected to standard RPI SPI pins.
