# test builtin ord (whether or not we support unicode)

print(ord('a'))

try:
    ord('')
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

# bytes also work in ord

print(ord(b'a'))
print(ord(b'\x00'))
print(ord(b'\x01'))
print(ord(b'\x7f'))
print(ord(b'\x80'))
print(ord(b'\xff'))

try:
    ord(b'')
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

# argument must be a string
try:
    ord(1)
    print("FAIL")
    raise SystemExit
except TypeError:
    print("PASS")
