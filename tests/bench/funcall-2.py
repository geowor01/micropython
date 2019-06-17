import time
num = 100000
t = time.ticks_ms()

def f(x):
    return x + 1

def test(num):
    for i in iter(range(num)):
        a = f(i)

test(num)
t = time.ticks_ms() - t
print("PASS: " + str(t))