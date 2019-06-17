import time
num = 100000
t = time.ticks_ms()
from ucollections import namedtuple

T = namedtuple("Tup", ["foo1", "foo2", "foo3", "foo4", "num"])

def test(num):
    t = T(0, 0, 0, 0, 200000)
    i = 0
    while i < t.num:
        i += 1

test(num)
t = time.ticks_ms() - t
print("PASS: " + str(t))