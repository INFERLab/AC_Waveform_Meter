import serial
import struct
from cobs import cobs
import numpy as np
import matplotlib.pyplot as plt

#  10/(2^16) = 0.000152587890625
#  20/(2^16) = 0.00030517578125
SCALE_FACTOR = 0.00030517578125

# the burden resistor is 8.25 ohms
NUM_CHANNELS = 8

# 2 bytes per channel + 1 '\n' byte
frameSize = 2 * NUM_CHANNELS + 1

# if USB cable is used /dev/ttyACM0, GPIO = /dev/ttyAMA0
ser = serial.Serial('/dev/ttyAMA0', 2000000)

synced = False # synchronization
running = True

while running:
    while not synced:
        print('Trying to sync')
        b = ser.read()
        print('Successfully read')
        if(b == 'a'):
            synced = True

    print('Successfully synced',b)

    while synced:

        num_samples = ser.read(2)
        num_samples = struct.unpack("<h", num_samples)[0]

        print(num_samples)
        data = ser.read(num_samples*16+1)
        print('last symb', data[-1])

        if data[-1] == 'a':
            
            all_vals = []
            for i in range(num_samples):
                values = []
                for j in range(8):
                    print(i*8 + j*2,i*8 + (j+1)*2)
                    v = struct.unpack(">h", data[i*16 + j*2:i*16 + (j+1)*2])[0]
                    values.append(v)
                all_vals.append(values)

            all_vals = np.array(all_vals)
            print(all_vals[:,0])
            plt.plot(all_vals[:,0])
            plt.plot(all_vals[:,1])
            plt.plot(all_vals[:,2])
            plt.plot(all_vals[:,3])
            plt.plot(all_vals[:,4])
            plt.plot(all_vals[:,5])
            plt.plot(all_vals[:,6])
            plt.plot(all_vals[:,7])
            plt.savefig('waveform.png')

            synced = False
            running = False
        
    else:
        synced = False



