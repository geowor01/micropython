import time
num = 100000
t = time.ticks_ms()

def test(num):
    zero = 0
    while num != zero:
        num -= 1

test(num)
t = time.ticks_ms() - t
print("PASS: " + str(t))