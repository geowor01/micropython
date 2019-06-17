import time
num = 100000
t = time.ticks_ms()

def test(num):
    for i in iter(range(num)):
        a = i + 1

test(num)
t = time.ticks_ms() - t
print("PASS: " + str(t))