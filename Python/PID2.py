import struct
from numpy import genfromtxt
import matplotlib.pyplot as plt
from sys import argv, path
from serial import Serial
from time import sleep
from os.path import isfile
from os import chdir

chdir(path[0])

tunings = [
    # (5,
    #  2,
    #  -0.028, 
    #  0),
    # (5,
    #  2,
    #  -0.028, 
    #  160),
    # (5,
    #  2,
    #  -0.028, 
    #  40),
    # (5,
    #  2,
    #  -0.028, 
    #  10),
    # Kp
    # (0.5, 0, 0, 0),
    # (2, 0, 0, 0),
    # (5, 0, 0, 0),
    # (10, 0, 0, 0),
    # Ki
    # (2, 0, 0, 0),
    # (2, 10, 0, 0),
    # (2, 20, 0, 0),
    # (2, 100, 0, 0),
    # Kd
    # (2, 20, 0, 0),
    # (2, 20, -0.01, 0),
    # (2, 20, -0.02, 0),
    # (2, 20, -0.05, 0),
    # Kp
    # (2, 20, -0.02, 0),
    # (5, 20, -0.02, 0),
    # (10, 20, -0.02, 0),
    # (20, 20, -0.02, 0),
    # Kd
    # (8, 10, -0.02, 0),
    # (8, 10, -0.03, 0),
    # (8, 10, -0.04, 0),
    # (8, 10, -0.06, 0),
    # fc
    # (9.5, 20, -0.05, 0),
    # (9.5, 20, -0.05, 100),
    # (9.5, 20, -0.05, 80),
    # (9.5, 20, -0.05, 30),

    # -------- 2 --------
    # Kp
    # (0.5, 0, 0, 0),
    # (2, 0, 0, 0),
    # (4, 0, 0, 0),
    # (10, 0, 0, 0),
    # Kd
    # (4, 0, 0, 0),
    # (4, 0, 0.01, 0),
    # (4, 0, 0.03, 0),
    # (4, 0, 0.06, 0),
    # Ki
    # (4, 0, 0.03, 0),
    # (4, 2, 0.03, 0),
    # (4, 5, 0.03, 0),
    # (4, 20, 0.03, 0),
    # fc
    # (4, 2, 0.03, 0),
    # (4, 2, 0.03, 100),
    # (4, 2, 0.03, 60),
    # (4, 2, 0.03, 10),
    # Kp
    # (4, 2, 0.03, 60),
    # (5, 2, 0.03, 60),
    # (6, 2, 0.03, 60),
    # (10, 2, 0.03, 60),
    # Kd
    # (6, 2, 0.03, 60),
    # (6, 2, 0.035, 60),
    # (6, 2, 0.04, 60),
    # (6, 2, 0.06, 60),
    # Ki
    # (6, 2, 0.04, 60),
    # (6, 4, 0.04, 60),
    # (6, 6, 0.04, 60),
    # (6, 8, 0.04, 60),
    # Final
    (6, 4, 0.04, 60),
    
    # -------- 3 --------
    # Ku
    # (5, 0, 0, 0),
    # (15, 0, 0, 0),
    # (19, 0, 0, 0),
    # (25, 0, 0, 0),
    # PID
    # (0.45 * 19, 1.2 * 19 / (31 * 960e-6), 3 * 19 * (31 * 960e-6) / 40, 100),
    # (0.45 * 19, 1.2 * 19 / (36 * 960e-6), 3 * 19 * (36 * 960e-6) / 40, 100),
    # (0.45 * 19, 0.9 * 19 / (36 * 960e-6), 3 * 19 * (36 * 960e-6) / 40, 100),
    # (0.45 * 19, 1.2 * 19 / (36 * 960e-6), 3 * 19 * (36 * 960e-6) / 40, 80),
    # (0.45 * 19, 0.9 * 19 / (36 * 960e-6), 3 * 19 * (36 * 960e-6) / 40, 80),
    # (0.45 * 19, 0.7 * 19 / (36 * 960e-6), 3 * 19 * (36 * 960e-6) / 40, 80),
    (0.45 * 19, 1.2 * 19 / (27 * 960e-6), 3 * 19 * (27 * 960e-6) / 40, 70),
    (0.45 * 19, 0.6 * 19 / (27 * 960e-6), 3 * 19 * (27 * 960e-6) / 40, 70),
    (0.45 * 19, 0.6 * 19 / (27 * 960e-6), 3.5 * 19 * (27 * 960e-6) / 40, 70),
]

speed = 8.

fader_idx = 0
sma = False
imgname = 'ziegler-nichols2.svg'

END = b'\300'
ESC = b'\333'
ESC_END = b'\334'
ESC_ESC = b'\335'

def write_slip(ser: Serial, data: bytes):
    print(data)
    slipdata = bytearray(END)
    for d in data:
        if   bytes([d]) == END: slipdata.extend(ESC + ESC_END)
        elif bytes([d]) == ESC: slipdata.extend(ESC + ESC_ESC)
        else: slipdata.append(d)
    slipdata.extend(END)
    ser.write(slipdata)

def read_slip(ser: Serial):
    buf = bytearray()
    escape = False
    while True:
        data = ser.read()
        if not data: return None
        elif data == END: 
            try: return struct.unpack("<hhh", buf)
            except: 
                if len(buf) > 1: print(buf)
            buf = bytearray()
            escape = False
        elif data == ESC: escape = True
        else:
            if escape: 
                if data == ESC_END: data = END 
                elif data == ESC_ESC: data = ESC
                escape = False
            buf.extend(data)


def get_tuning_name(tuning: tuple((float, float, float, float))):
    name = ''
    for i in range(0, 4):
        setting = ('p', 'i', 'd', 'c')[i]
        set_k = setting + str(tuning[i])
        name += set_k
    name += 's' + str(speed)
    if sma: name += '-sma'
    return name

def get_readable_name(tuning: tuple((float, float, float, float))):
    name = ''
    for i in range(0, 3):
        setting = ('p', 'i', 'd')[i]
        set_k = '$K_' + setting + ' = ' + str(tuning[i]) + '$,     '
        name += set_k
    name += '$f_c = ' + str(tuning[3]) + '$'
    # name += ' (speed: ' + str(speed) + ')'
    if sma: name += ' (SMA)'
    return name

def set_tuning(ser: Serial, tuning: tuple((float, float, float, float))):
    for i in range(0, 4):
        setting = ('p', 'i', 'd', 'c')[i]
        set_k = (setting + str(fader_idx)).encode()
        set_k += struct.pack('<f', tuning[i])
        write_slip(ser, set_k)
        sleep(0.01)

def start(ser: Serial):
    msg = b's0' + struct.pack('<f', speed)
    write_slip(ser, msg)

fig, axs = plt.subplots(len(tunings), 1, sharex='all', sharey='all', figsize=(12, 8), squeeze=False)
with Serial('/dev/ttyUSB0', 1_000_000, timeout=0.5) as ser:
    for i, tuning in enumerate(tunings):
        print(get_readable_name(tuning))
        filename = 'data' + get_tuning_name(tuning) + '.tsv'
        print(filename)
        if not isfile(filename):
            with open(filename, 'w') as f:
                ser.reset_input_buffer()
                if i != 0:
                    set_tuning(ser, (2, 10, 0.028, 40))
                    read_slip(ser)
                sleep(2.1) # allow the bootloader to finish
                set_tuning(ser, tuning)
                read_slip(ser)
                ser.reset_input_buffer()
                start(ser)
                while True:
                    data = read_slip(ser)
                    if data is None:
                        break
                    f.write('\t'.join(map(str, data)) + '\n')
                print()

        data = genfromtxt(filename, delimiter='\t')
        axs[i][0].plot(data, linewidth=1)
        axs[i][0].axhline(0, linewidth=0.5, color='k')
        axs[i][0].set_title(get_readable_name(tuning))

plt.xlim(left=246 * speed)
plt.xlim(right=len(data) - 200 * speed)
plt.tight_layout()
plt.savefig(imgname)
plt.show()
