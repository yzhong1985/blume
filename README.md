# blume
A long distance LoRa file transfer solution (work with relatively larger files), blume stands for **B**i-Directional, **L**ow-Powered, **U**ntethered, **M**ultimedia, **E**xporter

# usages
The program was developed to work with ESP32 board(could be other types) uses the Semtech SX series LoRa chip, the hardware tested is the Heltech ESP32 LoRa V2, below is a guide for setup the system
1. Load the sender/receiver program on the ESP32 board respectively, best way to load is to use the Arduino IDE
2. Copy the python sender/receiver program to the sender/receiver device(Raspberry Pi 4/Nvidia Jetson/PC)
3. Connect the ESP32 LoRa board to the sender/receiver device respectively
4. Start the receiver program. Usage: serial_file_receiver.py [-h] -p PORT [-c CHKSUM]
5. Start the sender program and specify a file to send. Usage: serial_file_sender.py [-h] -p PORT -f FILE

*Note that this program will need the python 3 (python 2 won't work) and pyserial will need to be installed*




