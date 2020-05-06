import serial
import sys
import time
import argparse
import os.path


def get_file_extension(code):
    support_files = {
        0x01: ".jpg",
        0x02: ".png",
        0x03: ".gif",
        0x04: ".txt",
        0x05: ".wav",
        0x06: ".mp3",
        0x07: ".mp4"
    }
    return support_files.get(code, "invalid file extension")


serialPort = 'COM4'                         # COM4/6
serialBaud = 115200                         # the serial speed (need to be given in args)
PACKET_SIZE = 255                           # the packet size (need to be set in args)
localSaveFolderPath = '/img/transfer/'      # set your file save folder
# imgFileExt = '.jpg'
imageBuff = bytearray()
headerChkBytesValue = 2020
isChecksum = False
isEndPacket = False
isOutput = False

fileBuff = []           # the packet will be save into this buff list
# errPackets = []         # for saving the err packets and request new resend request

# arguments setting
ap = argparse.ArgumentParser()
ap.add_argument("-p", "--port", required=True, help="specify the serial port to be listened")
ap.add_argument("-c", "--chksum", type=bool, default=False, help="if checksum will be examined")
args = vars(ap.parse_args())

# get the port number
serialPort = args["port"]

# if checksum indicator was not supplied,
if not args.get("chksum", False):
    isChecksum = False
# otherwise, set the checksum
else:
    isChecksum = args["chksum"]

isChecksum = False   # For debugging
headerCheck = bytearray(headerChkBytesValue.to_bytes(2, byteorder='big'))



try:
    s = serial.Serial(serialPort, serialBaud, timeout=4)
    print('Listening to the port: ' + serialPort)
    currentPath = os.path.abspath(os.path.dirname(__file__))
    idx = 0

    while True:
        if isOutput:  # output the image file to a folder
            ts = int(time.time())
            file_ext = get_file_extension(file_ext_code)
            outputFile = currentPath + localSaveFolderPath + str(ts) + file_ext
            with open(outputFile, mode='wb') as file:
                file.write(imageBuff)
                print('a file was received and saved at ' + outputFile)
            imageBuff.clear()
            isOutput = False
        dataPacket = s.read(PACKET_SIZE)  # dataPacket = s.readline()
        if len(dataPacket) > 0 and (dataPacket[0] == headerCheck[0]) and (dataPacket[1] == headerCheck[1]):
            idx = idx + 1
            # print(dataPacket)                 # for debug, if we want to examine the packet content
            id_dpk = dataPacket[2]*256 + dataPacket[3]
            total_dpk = dataPacket[4]*256 + dataPacket[5]
            file_ext_code = dataPacket[6]
            num_zero = 0
            if id_dpk + 1 == total_dpk:  # if last packet, count the trailing zeros, set the output to true
                isOutput = True
                for x in range(1, PACKET_SIZE):
                    c = dataPacket[PACKET_SIZE - x]
                    if c == 0:
                        num_zero = num_zero + 1
                    else:
                        break

            imgLastByteIndex = PACKET_SIZE - 2 - num_zero  # packet size - two bytes of checksum - num of zeros

            # below is to check the checksum to confirm the end-to-end data integrity
            validate = ''
            if isChecksum:
                checksum = sum(dataPacket[0:imgLastByteIndex])
                if ((checksum // 256) == dataPacket[imgLastByteIndex]) and \
                        ((checksum % 256) == dataPacket[imgLastByteIndex+1]):
                    validate = ' valid:Yes'
                    # print('checksum is correct!')
                    # fileBuff.append(dataPacket)
                else:
                    validate = ' valid:No'
            else:
                # fileBuff.append(dataPacket)
                validate = ''

            msg = 'received packet #:' + str(idx) + ' size:' + str(len(dataPacket)) + \
                  ' progress:' + str(id_dpk + 1) + '/' + str(total_dpk) + validate
            print(msg)
            s.write((msg + '\r').encode())

            # only if we dont do checksum
            imageBuff.extend(dataPacket[8:imgLastByteIndex])    # skip the first 8 bytes (header, index, total)
            # print(len(imageBuff))                             # for debug, examine the aggregated bytes of the image

    s.close()
    print('disconnected!')
except:
    print("Failed to connect with " + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
    print("Unexpected error:", sys.exc_info()[0])
    print("Unexpected error:", sys.exc_info()[1])
    exc_tb = sys.exc_info()[2]
    print("Line# " + str(exc_tb.tb_lineno))




