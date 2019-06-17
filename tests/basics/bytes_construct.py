# test construction of bytes from different objects

# tuple, list, bytearray
print(bytes((1, 2)))
print(bytes([1, 2]))
print(bytes(bytearray(4)))

# constructor value out of range
try:
    bytes([-1])
    print("FAIL")
    raise SystemExit
except ValueError:
    pass

# constructor value out of range
try:
    bytes([256])
    print("FAIL")
    raise SystemExit
except ValueError:
    pass

# error in construction
try:
    a = bytes([1, 2, 3], 1)
    print("FAIL")
    raise SystemExit
except TypeError:
    print("PASS")