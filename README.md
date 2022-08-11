# nucleo-project

4 - led handler controlled on mobile app via bluetooth.

### Board used

[NUCLEO-L476RG](https://www.st.com/en/microcontrollers-microprocessors/stm32l476rg.html) board

### Board arrangement

Conect leds to pins PA1, PA6, PC0, PC1. Connect bluetooth module HC-06: TXD to PA10, RXD to PA9, as showed below.

TBD

### Mobile app

Follow this link to download the app. Install it on the Android device. Ignore security warnings.

### Requirements

To compile the project ***gcc-arm-none-eabi*** compiler is needed. To install it, run:
```bash
sudo apt install gcc-arm-none-eabi
```

### Building the project

Run ```make``` in the terminal.

### Flashing the firmware

Connect NUCLEO board with USB and move the .bin file from build catalogue to the device catalogue (NODE_L476RG).

### Usage example

TBD
