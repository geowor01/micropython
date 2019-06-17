import time
ITERS = 100000
t = time.ticks_ms()
for i in iter(range(ITERS//10000)):
    arr = [0] * 1000
    for i in range(len(arr)):
        arr[i] += 1
t = time.ticks_ms() - t
print("PASS: " + str(t))