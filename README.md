## What it is

Control video conferences thanks to a minimalist board based on the extra super cheap CH552g Chinese chip.

## How it works

There is no magic: the code simply simulates a keyboard (USB CDC mode), and it sends the application-specific shortcuts to the host.
It means that the *active* window will receive the keys codes, whichever it is (hopefully you will press the buttons only when the video conference is active).

Actually, you can use the CH552g to implement any kind of keyboard shortcut (like Control-C / Control-V)...

## Requirements:

### Sofware

This project uses a *copy* of [CH55xduino](https://github.com/DeqingSun/ch55xduino).

I do this with embedded electronics in order to avoid breaking my project when the external library change. It also means bug fixes are not always ported.
So yes, I might better use a git subproject.

### Hardware

The impressive CH552g only needs two 100nF caps on V33OUT and VCC, and obviously the 4 USB signals.

It runs both on 5V and 3V3 (it includes a 3V3 LDO). But I think 5V is required in order to flash it, so beware of your setup if you add other components.

The CH552g supports DFU, and/but the CH55xduino library will install its own bootloader on the first time you flash the chip.

So if you ever need to flash the bootloader again, the way out is to add two push buttons on your board (normally open):

  * RST: with a 1K resistor in serie towards 3V3, on pin 6 (RST)

  * PRG: with a 10K resistor in serie towards 2V2, on USB D+ ie. pin 12 (P3.6 UDP)

To put the chip in flash mode : reset, press and keep RST and PRG, release RST, wait a few seconds and release PRG.
