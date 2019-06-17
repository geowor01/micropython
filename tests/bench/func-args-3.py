import time
num = 100000
t = time.ticks_ms()

def func(a, b=1, c=2):
    pass

def test(num):
    for i in iter(range(num)):
        func(i)

test(num)
t = time.ticks_ms() - t
print("PASS: " + str(t))