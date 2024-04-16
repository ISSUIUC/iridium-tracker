import serial
from datetime import datetime

baud_rate = 115200
com_port = '/dev/cu.usbserial-2130'

listener = serial.Serial(com_port, baud_rate)

def write_to_file(arr):
    with open('Processing/logs/raw/log2.txt', 'a') as f:
        for line in arr:
            f.write(str(line))
            f.write("\n")

ar = []
t = 0
while 1:
    into = listener.readline()
    print(into)
    ar.append((str(datetime.now()), into))
    t += 1

    if t%10 == 0:
        write_to_file(ar)
        ar = []


