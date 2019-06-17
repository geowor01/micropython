import time
num = 100000
t = time.ticks_ms()
from ucollections import namedtuple

T = namedtuple("Tup", ["num", "bar"])

def test(num):
    t = T(200000, 0)
    i = 0
    while i < t.num:
        i += 1

test(num)
t = time.ticks_ms() - t
print("PASS: " + str(t))