import time
num = 100000
t = time.ticks_ms()
for i in iter(range(num//10000)):
    arr = bytearray(b"\0" * 1000)
    arr2 = bytearray(map(lambda x: x + 1, arr))
t = time.ticks_ms() - t
print("PASS: " + str(t))