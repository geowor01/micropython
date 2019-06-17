import time
num = 100000
t = time.ticks_ms()
for i in iter(range(num//10000)):
    ba = bytearray(b"\0" * 1000)
    for i in range(len(ba)):
        ba[i] += 1
t = time.ticks_ms() - t
print("PASS: " + str(t))