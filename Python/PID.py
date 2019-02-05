from numpy import genfromtxt
import matplotlib.pyplot as plt
from sys import argv, path
from serial import Serial
from time import sleep
from os.path import isfile
from os import chdir

chdir(path[0])

tunings = [
    # (2,
    #  2.5,
    #  -1e-2),
    # (4,
    #  2.5,
    #  -3e-2),
    (4,
     10,
     -3e-2),
    # (5,
    #  8,
    #  -1e-2),
    # (1,
    #  2,
    #  -8e-3),
    # (2,
    #  0.6,
    #  -9e-3),
    # (1.8,
    #  0.4,
    #  -9e-3),
    # (1.5,
    #  0.4,
    #  -1e-2),
    # (1.5,
    #  0.6,
    #  -1e-2),
    # (1.5,
    #  2.5,
    #  -1e-2),
    # (2.1,
    #  2.5,
    #  -1.21e-2),
    # (2.1,
    #  2.5,
    #  -1.41e-2),
    # (2.1,
    #  2.5,
    #  -1.91e-2),
    # (2.1,
    #  3.5,
    #  -1.91e-2),
    (2.1,
     4.5,
     -1.91e-2),
    (2.2,
     5.5,
     -1.91e-2),
    
    (2.6,
     5.5,
     -1.9e-2),
    # (2,
    #  1.5,
    #  -5e-3),
    # (2.5,
    #  0.5,
    #  -5e-3),
    # (3,
    #  0.5,
    #  -1e-2),
    # (5,
    #  1.5,
    #  -3e-2),
]

ACK = b'\x06'
EOT = b'\x04'

def get_tuning_name(tuning: tuple((float, float, float))):
    name = ''
    for i in range(0, 3):
        setting = ('p', 'i', 'd')[i]
        set_k = setting + str(tuning[i])
        name += set_k
    return name


def set_tuning(ser: Serial, tuning: tuple((float, float, float))):
    for i in range(0, 3):
        setting = ('p', 'i', 'd')[i]
        set_k = setting + str(tuning[i])
        print(set_k)
        ser.write(bytes(set_k + '\r\n', 'ascii'))
        response = ser.read()
        if (response != ACK):
            raise Exception('No ack for ' + setting + ' ('+str(response)+')')


for i in range(len(tunings)):
    tuning = tunings[i]
    filename = 'data' + get_tuning_name(tuning) + '.tsv'
    if not isfile(filename):
        with Serial('/dev/ttyACM0', 115200, timeout=1) as ser,\
            open(filename, 'wb') as f:
            ser.reset_input_buffer()
            sleep(2.1) # allow the bootloader to finish
            set_tuning(ser, tuning)
            ser.write(b'g1\r\n')
            response = ser.read()
            if (response != ACK):
                raise Exception('No ack for g1 ('+str(response)+')')
            while True:
                data = ser.read()
                if (data == EOT):
                    break
                f.write(data)
            print()

    plt.subplot(len(tunings), 1, 1 + i)
    data = genfromtxt(filename, delimiter='\t')
    plt.plot(data)

plt.tight_layout()
plt.show()
