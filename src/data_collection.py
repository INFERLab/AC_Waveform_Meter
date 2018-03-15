import serial
import struct
from cobs import cobs

#  10/(2^16) = 0.000152587890625
#  20/(2^16) = 0.00030517578125
SCALE_FACTOR = 0.00030517578125

# the burden resistor is 8.25 ohms
NUM_CHANNELS = 8

# 2 bytes per channel + 1 '\n' byte
frameSize = 2 * NUM_CHANNELS + 1

# if USB cable is used /dev/ttyACM0, GPIO = /dev/ttyAMA0
ser = serial.Serial('/dev/ttyAMA0', 2000000)

# save measurements
text_file = open("test03_13.txt", "w")

synced = False # synchronization

while not synced:
    print('Trying to sync')
    bit = ser.read()
    print(bit) 
    if(bit == '\x00'):
	synced = True

print('Successfully synced')

# infinite loop
while 1:
    print('Trying to read')
    frame = ser.read(frameSize + 1) # so we read the next 0
    print('Read')
    # after a 0, comes 5 bytes, 1OH and 4DATA
    data = cobs.decode(frame[0:frameSize])
    value1 = struct.unpack(">h", data[0:2])[0]
    value2 = struct.unpack(">h", data[2:4])[0]
    data1 = float(value1) * SCALE_FACTOR
    data2 = float(value2) * SCALE_FACTOR
    #print str(data1) + "," + str(data2) # debug purpose
    print(data1,data2)

text_file.close()
