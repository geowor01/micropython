import time
num = 100000
t = time.ticks_ms()
for i in iter(range(num//10000)):
    ba = bytearray(b"\0" * 1000)
    ba2 = bytearray(map(lambda x: x + 1, ba))
t = time.ticks_ms() - t
print("PASS: " + str(t))