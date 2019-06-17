# test [...] of bytes

print(b'123'[0])
print(b'123'[1])
print(b'123'[-1])

try:
    b'123'[1] = 4
    print("FAIL")
    raise SystemExit
except TypeError:
    print("PASS")

try:
    del b'123'[1]
    print("FAIL")
    raise SystemExit
except TypeError:
    print("PASS")
