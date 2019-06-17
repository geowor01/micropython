import time
num = 100000
t = time.ticks_ms()

def func(a, b, c):
    pass

def test(num):
    for i in iter(range(num)):
        func(c=i, b=i, a=i)

test(num)
t = time.ticks_ms() - t
print("PASS: " + str(t))