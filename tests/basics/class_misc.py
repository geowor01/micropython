# converting user instance to buffer
class C:
    pass

c = C()
try:
    d = bytearray(c)
    print("FAIL")
    raise SystemExit
except TypeError:
    print("PASS")
