"""
The program is to construct the data packets from supported file format then push it to the ESP32 LoRa via serial port
The supported file extension includes, the file size has to be smaller than 90 KB
    1 - jpg
    2 - txt
    3 - mp4
    .....

USAGE: python serialImageSender.py -p COM8 -f \img\cgu.jpg
"""

import sys
import time
import serial
import argparse
import os.path


def get_file_code(extension):
    support_extensions = {
        ".jpg": 0x01,
        ".png": 0x02,
        ".gif": 0x03,
        ".txt": 0x04,
        ".wav": 0x05,
        ".mp3": 0x06,
        ".mp4": 0x07
    }
    return support_extensions.get(extension, 0x00)


PACKET_SIZE = 255
PAYLOAD_SIZE = 245              # define the maximum image size (Max 255-10=245)
START_MSG = '##STRT#'           # define a message to notify the start of sending a series of packets
END_MSG = '##END##'             # define a message to notify the end

serialPort = ''                 # set based on local serial port name
serialBaud = 115200             # the baud for serial
filePath = ''                   # set your file path cgu_sm.jpg 320p_30cp.jpg
headerChkBytesValue = 2020
serialDelay = 0.2

# arguments setting
ap = argparse.ArgumentParser()
ap.add_argument("-p", "--port", required=True, help="specify the serial port to be listened")
ap.add_argument("-f", "--file", required=True, help="specify the filepath")
args = vars(ap.parse_args())
#
# # get the port number
serialPort = args["port"]
filePath = args["file"]

currentPath = os.path.abspath(os.path.dirname(__file__))
filePath = currentPath + filePath

try:
    with open(filePath, mode='rb') as file:
        data = file.read()

    filename, file_extension = os.path.splitext(filePath)
    extension_code = get_file_code(file_extension)
    print("the file to send - size:" + str(len(data)))
    imgPackets = [data[i:i+PAYLOAD_SIZE] for i in range(0, len(data), PAYLOAD_SIZE)]
    total = len(imgPackets)
    print("Total # of Packets:" + str(total))
    s = serial.Serial(serialPort, serialBaud, timeout=1)
    time.sleep(1)
    startSignal = bytearray(START_MSG.encode())
    n = PACKET_SIZE - len(startSignal)
    startSignal.extend([0x00] * n)          # need to add trailing zero to make it as a 250 bytes packet
    s.write(startSignal)                    # notify the start

    for j in range(0, total, 1):
        dataPacket = bytearray(headerChkBytesValue.to_bytes(2, byteorder='big'))    # the header check byte
        dataPacket.extend(j.to_bytes(2, byteorder='big'))                           # the current index of packet
        dataPacket.extend(total.to_bytes(2, byteorder='big'))                       # the total number of packets
        dataPacket.extend([extension_code])                                         # the file formats code
        dataPacket.extend([0x00])                                                   # reserve for multiple part of file
        dataPacket.extend(imgPackets[j])    # append the image content part
        checksum = sum(dataPacket)          # calculate the checksum
        dataPacket.extend((checksum // 256, checksum % 256))    # append the checksum
        print(dataPacket)
        packetIndex = dataPacket[2] * 256 + dataPacket[3]
        print("Packet index:" + str(packetIndex) + " size:" + str(len(dataPacket)))
        if len(dataPacket) < PACKET_SIZE:
            m = PACKET_SIZE - len(dataPacket)
            dataPacket.extend([0x00] * m)
        # s.write(dataPacket)                 # push packet to serial port
        time.sleep(serialDelay)

    time.sleep(0.5)
    endSignal = bytearray(END_MSG.encode())
    n = PACKET_SIZE - len(endSignal)
    endSignal.extend([0x00] * n)          # need to add trailing zero to make it 250 bytes
    s.write(endSignal)               # notify it is finished
    s.close()
except:
    print("Failed to connect with " + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
    print("Unexpected error:", sys.exc_info()[0])
    print("Unexpected error:", sys.exc_info()[1])
    exc_tb = sys.exc_info()[2]
    print("Line# " + str(exc_tb.tb_lineno))





