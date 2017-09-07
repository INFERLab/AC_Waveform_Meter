import matplotlib.pyplot as plt

fileName = '/media/matias/ExtraDrive1/CEE-RA/Electrical Power Meter/test08_12_PAPER.txt'
dataset = open(fileName, 'r')

d1 = []
d2 = []
d3 = []
d4 = []

for line in dataset:
    if len(line.split(',')) > 1:
        d1.append(float(line.split(',')[0]) * 25.76) # AC-AC adapter ratio
        d2.append(float(line.split(',')[1]) * 100) # Fluke scaling ratio
        d3.append(float(line.split(',')[2]) * 61) # TED CT ratio
        print line.split(',')
        #d4.append(float(line.split(',')[3]))
#plt.plot(d1,label = 'Voltage')    
plt.plot(d2,label = 'Fluke i200 CT')
plt.plot(d3,label = 'TED CT')
#plt.plot(d4,label = 'Current3')

plt.legend()
plt.show()
#for latex plots
#print d2