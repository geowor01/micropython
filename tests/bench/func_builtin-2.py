import time
num = 100000
t = time.ticks_ms()

def test(num):
    for i in iter(range(num//20)):
        enumerate([1, 2], start=1)

test(num)
t = time.ticks_ms() - t
print("PASS: " + str(t))