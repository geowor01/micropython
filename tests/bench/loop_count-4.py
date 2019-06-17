import time
num = 100000
t = time.ticks_ms()

def test(num):
    while num > 0:
        num -= 1

test(num)
t = time.ticks_ms() - t
print("PASS: " + str(t))