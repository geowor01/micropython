import time
num = 100000
t = time.ticks_ms()

ITERS = 200000

def test(num):
    i = 0
    while i < ITERS:
        i += 1

test(num)
t = time.ticks_ms() - t
print("PASS: " + str(t))