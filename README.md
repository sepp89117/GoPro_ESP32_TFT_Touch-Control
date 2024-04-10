# GoPro_ESP32_TFT_Touch-Control
Allows control and monitoring of four cameras at the same time
<br><br>
Look also https://github.com/sepp89117/GoEasyPro_Android
<br><br>
Tested with GoPro Hero 5 black an Hero 8 black

<img src="https://raw.githubusercontent.com/sepp89117/GoPro_ESP32_TFT_Touch-Control/main/show.jpg">

Using TFT_eSPI from https://github.com/Bodmer/TFT_eSPI
<br>

I use it with an ILI9341 320x240 TFT display with SPI

A short demonstration on YouTube: https://youtu.be/x3VdKUKrlSc

## Installation
1. Download and extract the source into Arduino projects folder
2. Download and install TFT_eSPI from https://github.com/Bodmer/TFT_eSPI
3. Config TFT_eSPI (docs: https://github.com/Bodmer/TFT_eSPI/blob/master/docs/ESP-IDF/Using%20ESP-IDF.txt)
    - i have commented out ``#include <User_Setup.h>`` and uncommented ``#include <User_Setups/Setup42_ILI9341_ESP32.h>`` in ``User_Setup_Select.h`` and ``#define TOUCH_CS 5`` in ``Setup42_ILI9341_ESP32.h``
4. Make sure your wiring matches the pin definitions in in ``Setup42_ILI9341_ESP32.h`` or in your own TFT_eSPI setup
5. Install esp32 with your board manager in Arduino IDE (actually version 2.0.14)
6. In Arduino IDE select your ESP32 board and the COM port connected to
7. Compile and upload
