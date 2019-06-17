import time
num = 100000
t = time.ticks_ms()

def test(num):
    i = 0
    while i < 200000:
        i += 1

test(num)
t = time.ticks_ms() - t
print("PASS: " + str(t))