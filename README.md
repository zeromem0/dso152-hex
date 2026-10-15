# Tester RS232 for the Fnirsi DSO152

Alternative firmware for the FNIRSI DSO-152: RS232/TTL serial tester, TTL sender and flash viewer.

Read an RS232 line **directly via the BNC connector**, through the oscilloscope's front-end,
without a USART or a MAX3232. It displays the waveform, the characters aligned beneath
the bits from which they were derived, and the data stream in both hex and ASCII.

> **Note:** the initial work comes from David's
> [dso152-breakout](https://github.com/davidgiven/dso152-breakout).

The buttons are described in [docs/buttons.html](docs/buttons.html)
([Romanian](docs/butoane.html)).

## Ready-made binary

[`usb/DSO152_APP_V0.1.bin`](usb/DSO152_APP_V0.1.bin) is a build of this firmware (Hex v0.88).
It can be installed as it is, see [Installing](#installing-through-the-usb-drive).

## Building

You need [PlatformIO](https://platformio.org/): the VS Code extension, or the command line
(`pip install platformio`). The platform (`ststm32@19.0.0`) and the library (Adafruit GFX
1.12.6) are pinned in `platformio.ini` and are downloaded on the first build.

```
git clone https://github.com/zeromem0/dso152-hex.git
cd dso152-hex
pio run -e dso152-bl
```

The result is `.pio/build/dso152-bl/firmware.bin`.

**Build `dso152-bl`, not the default environment.** `dso152-bl` is linked at `0x08003C00`,
above the FNIRSI bootloader, where the bootloader expects the application. The default
`dso152` is linked at `0x08000000` and is only for a unit flashed over SWD without a
bootloader. Put on the USB drive, it would not start.

Other environments: `dso152-bl-demo` feeds a synthetic 9600 baud signal instead of the
ADC (for a unit with nothing on the input). `dso152-bl-shot` and `dso152-bl-selftest` are
test builds that need a debugger attached. **Do not install these on a unit.**

## Installing through the USB drive

The DSO-152 has a bootloader that shows up as a USB drive. Nothing needs to be opened,
and the bootloader itself is never touched.

1. Rename the binary to exactly **`DSO152_APP_V0.1.bin`**. The one in `usb/` already has that name.
2. Switch the unit off.
3. Connect it to the computer with a USB cable.
4. **Hold OK and switch it on.** The screen shows *Enter firmware upgrade mode*, and a USB
   drive appears on the computer.
5. **Drag and drop** `DSO152_APP_V0.1.bin` onto that drive and wait until the copy has finished.
6. Eject the drive, unplug, and switch the unit off and on. The start menu should appear.

Good to know:

- Once this firmware is installed, the text on the upgrade screen is garbled. The bootloader
  takes its font from the application's area of flash. It is harmless: the drive still works.
- If the unit stays dark or the computer reports *Device Descriptor Request Failed*, cut the
  power for real (unplug the USB cable and wait until it is off), then start again from step 2.
- **Going back to the oscilloscope:** install the FNIRSI firmware file the same way.
- **The FNIRSI calibration is kept.** This firmware stores its own settings only in the last
  16 bytes of the calibration page, and rewrites the rest of that page as it was. If the
  stock firmware ever lacks a calibration, recalibrate it: hold OK until the *Calib* window
  appears, then press RUN.

## Receiving an export on a laptop

TTL Sender puts the buffer, or a stretch of flash, on the calibration output as 3.3 V serial
8N1. Connect a USB-TTL adapter: RX to the calibration output, ground to ground. Then run:

```
powershell -ExecutionPolicy Bypass -File tools\ttl_receive.ps1 -ListPorts
powershell -ExecutionPolicy Bypass -File tools\ttl_receive.ps1 -Port COM5 -Baud 9600
```

The script writes `received.bin` and prints a CRC-32. It must match the CRC shown on the
instrument's screen.
