import time
num = 200000
t = time.ticks_ms()

def test(num):
    i = 0
    while i < num:
        i += 1

test(num)
t = time.ticks_ms() - t
print("PASS: " + str(t))