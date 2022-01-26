import serial
from random import randint

PORT1 = 'COM8'
PORT2 = 'COM17'
SPEED1 = 9600
SPEED2 = 115200

NUM_OF_BYTES = 128

port1 = serial.Serial(
                    port=PORT1,
                    baudrate=SPEED1,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS,
                    timeout=1.1)

port2 = serial.Serial(
                    port=PORT2,
                    baudrate=SPEED2,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS,
                    timeout=1.1)


line = ''
for i in range(NUM_OF_BYTES):
    line += str(randint(0, 9))

#print('send')
data = bytes(line, encoding='utf-8')
print(data)

port1.write(data)


#print('receive')
answer = port2.read(NUM_OF_BYTES)


print(answer)

if data == answer:
    print('OK')
else:
    print('Err')




