# Dichotomy Keyboard Firmware
This is the code for the nRF51822 modules used in Dichotomy - the hands and the receiver.  There are also pre-compiled hex files that can be flashed directly if you don't care about building it yourself.  This was done with SDK V11, so.... dunno if it'll work with other versions.

## Install dependencies

I did this all on Ubunto 16.04.

```
sudo apt install openocd gcc-arm-none-eabi
```

## Download Nordic SDK

Nordic does not allow redistribution of their SDK or components, so download and extract from their site:

https://developer.nordicsemi.com/nRF5_SDK/

Firmware written and tested with version 11

## Notes:

A bunch of this readme is copied from Mitosis, of which Dichotomy is a derivative.  Looking there might provide other information.

https://github.com/reversebias/mitosis

## Toolchain set-up

A cofiguration file that came with the SDK needs to be changed. Assuming you installed gcc-arm with apt, the compiler root path needs to be changed in /components/toolchain/gcc/Makefile.posix, the line:
```
GNU_INSTALL_ROOT := /usr/local/gcc-arm-none-eabi-4_9-2015q1
```
Replaced with:
```
GNU_INSTALL_ROOT := /usr/
```

## Clone repository
Inside nRF5_SDK_11/
```
git clone https://github.com/Snipeye/Dichotony-Nordic
```

## Install udev rules
```
sudo cp Dichotomy/49-stlinkv2.rules /etc/udev/rules.d/
```
Plug in, or replug in the programmer after this.

## OpenOCD server
The programming header on the side of the keyboard, from top to bottom:
```
SWCLK
SWDIO
GND
3.3V
```
It's best to remove the battery during long sessions of debugging, as charging non-rechargeable lithium batteries isn't recommended.

Launch a debugging session with:
```
openocd -f Dichotomy/nrf-stlinkv2.cfg
```
Should give you an output ending in:
```
Info : nrf51.cpu: hardware has 4 breakpoints, 2 watchpoints
```
Otherwise you likely have a loose or wrong wire.


## Manual programming
From the factory, these chips need to be erased:
```
echo reset halt | telnet localhost 4444
echo nrf51 mass_erase | telnet localhost 4444
```
From there, the precompiled binaries can be loaded:
```
telnet localhost 4444
reset halt
flash write_image erase /home/path/to/the/file/left.hex
reset
exit
```
