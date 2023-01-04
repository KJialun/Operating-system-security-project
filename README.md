# Operating-system-security-project
Arduino/Esp32/freertos/MQTT/AES encryption/authentication &amp; integrity check

## Document description :

- CP210x_Universal_Windows_Driver : ESP32 driver 
- OS_MULTI_TASK_Schedule          : Single Esp32 communicates with the server  
- Two_esp_comm                    : Two Esp32 communication through the MQTT server 
- lib                             : The libraries used in this project 
- mosquitto                       : MQTT server Software
- ESP32_Single_Core.ino           : Performance test by using single core 
- ESP32_Dual_Core.ino             : Performance test by using Dual core


## ESP32 Hardware Schematic  
![ELEC-H423_Schematic](https://user-images.githubusercontent.com/121833181/210653659-dc7ec7e6-5297-4bad-ac72-5906f5f74ae3.png)

## Useful reference link 

- Connect your ESP32 to the Arduino IDE:

https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/

- Additional boards manager URLs: 

https://dl.espressif.com/dl/package_esp32_index.json 

https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

- Make your first program using the arduino IDE:

https://docs.arduino.cc/built-in-examples/basics/Blink

- Esp32 - AES encryption & decryption  

https://techtutorialsx.com/2018/04/18/esp32-arduino-encryption-using-aes-128-in-ecb-mode/

https://techtutorialsx.com/2018/05/10/esp32-arduino-decrypt-aes-128-in-ecb-mode/

- Using Padding in AES Encryption (PKCS5 Padding Method):

https://www.di-mgt.com.au/cryptopad.html

- Hash standard:

https://en.wikipedia.org/wiki/SHA-2

- Hash-based Message Authentication Code (HMAC)  for Authentication  : 

https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/api-reference/peripherals/hmac.html

- Running a Buffer Overflow Attack (Understand what is Overflow attack and how ?)

https://www.youtube.com/watch?v=1S0aBV-Waeo

- Dynamic memory allocation in C - malloc() / calloc() / realloc() / free() / (Understand how to allocate heap meamory? tutorial video)

https://www.youtube.com/watch?v=xDVC3wKjS64

- Setting MQTT server on windows by using mosquitto  (tutorial video):

https://www.youtube.com/watch?v=DH-VSAACtBk

- Esp32 - DH11 Sensor - MQTT - Node-red application code (tutorial example):

https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/

- Node-red tutorial video: 

https://www.bilibili.com/video/BV14G4y1h7aJ/?spm_id_from=333.1007.top_right_bar_window_history.content.click&vd_source=4cdbb5ede3d02f6722fb42ac860bd41c

- Freertos base on esp32 tutorial videos: 

https://www.youtube.com/watch?v=F321087yYy4&list=PLEBQazB0HUyQ4hAPU1cJED6t3DU0h34bz

- Peformance benchmarking with FREERTOS | ADVANCED ESP32:

https://www.youtube.com/watch?v=WxnCP7HwWjc
