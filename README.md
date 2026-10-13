# 10cscope
Digital Sampling Oscilloscope on a ten cent MCU.

* Sampling rate: 600kHz / 150kHz
* Freerun / rising edge trigger
* 0-3.3V input range
* 31 x 16 scope "display"

# How to build

Use MounRiver Studio II (aka MRS2) to open 10cscope project file (`10cscope.wvproj`) then build it (`F7`).

# How to use

## terminal

Use your favorite terminal emulator software (this project was developed and tested with putty on Windows) and usb-serial converter to connect to the MCU at 115200N81.

## user interface

Press `Ctrl-R` to refresh the interface. Use cursor keys to move trigger horizontal position and trigger level. Enjoy.

## pins

`CH32V003A4M6` in a `SOIC-8` package with the following pins used:

```
pin    type    description  

PC2    GPIO    virtual ground for analog input
PC4    ADC1    CH2
PD6    USART1  TX
PD5    USART1  RX
```

