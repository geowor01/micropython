# test errors operating on bignum

i = 1 << 65

try:
    i << -1
    print("FAIL")
    raise SystemExit
except ValueError:
    pass

try:
    len(i)
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

try:
    1 in i
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

# overflow because arg of bytearray is being converted to machine int
try:
    bytearray(i)
    print("FAIL")
    raise SystemExit
except OverflowError:
    pass

# to test conversion of negative mpz to machine int
# (we know << will convert to machine int, even though it fails to do the shift)
try:
    i << (-(i >> 40))
    print("FAIL")
    raise SystemExit
except ValueError:
    pass

try:
    i // 0
    print("FAIL")
    raise SystemExit
except ZeroDivisionError:
    pass

try:
    i % 0
    print("FAIL")
    raise SystemExit
except ZeroDivisionError:
    print("PASS")
