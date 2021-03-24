import numpy as np

STM_FREQ = 1000000
SIN_FREQ = 1000
AMPLITUDE = 1023
OFFSET = 2048

N = int(STM_FREQ/SIN_FREQ)
f = open("result.txt", 'w')
line = "Frequency: %d Hz\n" % 10000
f.write(line)
line = "NUM: %d\n\n    " % N
f.write(line)
for i in range(N):
    value = round( AMPLITUDE * np.sin(2 * np.pi * i / N) + OFFSET )
    line = "%d, " % value
    f.write(line)
    if i%10 == 9:
        f.write("\n    ")
f.close()
