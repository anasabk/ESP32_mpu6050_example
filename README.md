# An Example for Using An MPU6050 sensor with ESP32 Boards
This is an example showcasing an a usecase of the MPU6050 sensor connected to an ESP32 board. The program is implemented in the ESP-idf, and the [I2Cdev](https://github.com/jrowberg/i2cdevlib/tree/master/ESP32_ESP-IDF) library was used to acheive communication with the sensor. The data is collected from the sensor, then sent through a UDP client to a UDP server working on a computer, and also printed on the serial monitor connected to the ESP32.

### To launch the example
 - Add your Wifi's SSID and password through the configuration settings.
 - Have a UDP server on your computer.
 - Write the UDP server's IP address and port next to the macros HOST_IP_ADDR and DEST_PORT.
 - Compile and flash.
