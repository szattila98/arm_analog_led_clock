# ARM analog led clock
Program for the Architectures and Embedded Developement Systems assignment, at the University of Miskolc.

## Inner workings
It's made with CubeIDE and uses a rotary encoder and a WS2812 protocol LED ring, with 12 leds. The microcontroller is a STM32F103-MS, or "blue pill".
The LED ring is controlled with SPI, a hexadecimal bit array is sent, containing the colors of all leds.

## How should it function
The leds are the pointers of a clock, every pointer is a different color. The encoder can be use to set the time, after pressing it down, it can be cycled trough the time change modes and the pointer positions can be changed by rotating the encoder. To see debug info, connect the USB port to a PC and use a software to view the messages, like [Tera Term](https://ttssh2.osdn.jp/index.html.en).
