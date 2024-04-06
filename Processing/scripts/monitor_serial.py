import serial

baud_rate = 115200
com_port1 = '/dev/cu.usbserial-2140'

listener = serial.Serial(com_port1, baud_rate)

def write_to_file(arr):
    with open('../../logs/log.txt', 'a') as f:
        for line in arr:
            f.write(str(line))
            f.write("\n")

ar = []
t = 0
while 1:
    into = listener.readline()
    print(into)
    ar.append(into)
    t += 1

    if t%10 == 0:
        write_to_file(ar)
        ar = []


