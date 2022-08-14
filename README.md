# nucleo-project

4 - led handler controlled on mobile app via bluetooth.

### Board used

[NUCLEO-L476RG](https://www.st.com/en/microcontrollers-microprocessors/stm32l476rg.html) board

### Board arrangement

Conect leds handled by app to pins PA1, PC1, PA0, PA6. Connect bluetooth module HC-06: TXD to PA10, RXD to PA9 (remember to decrese the voltage using transistors - from 5V to 3,3V). Connect potentiometer to PC0. As showed below.

![Screenshot from 2022-08-14 22-01-24](https://user-images.githubusercontent.com/71830127/184552956-a7628e03-165f-43e4-bec6-fef5f8e80d06.png)

### Mobile app

On the Android device open .apk file placed in mobile-app folder. Ignore security warnings.

App made with [MIT App Inventor](https://appinventor.mit.edu/)

### Requirements

To compile the project ***gcc-arm-none-eabi*** compiler is needed. To install it, run:
```bash
sudo apt install gcc-arm-none-eabi
```

### Building the project

Run ```make``` in the terminal.

### Flashing the firmware

Connect NUCLEO board with USB and move the .bin file from build catalogue to the device catalogue (NODE_L476RG).

### Photos

<img src="[http://....jpg](https://user-images.githubusercontent.com/71830127/184553153-f5b9bbcc-89a7-4485-b998-59e9f990f706.jpg)" width="200" height="200" />

![usage2](https://user-images.githubusercontent.com/71830127/184553153-f5b9bbcc-89a7-4485-b998-59e9f990f706.jpg) ![usage1](https://user-images.githubusercontent.com/71830127/184553155-6045d31c-7e1c-47e4-b146-c7686cd9525b.png)




