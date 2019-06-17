# basic tuple functionality
x = (1, 2, 3 * 4)
print(x)
try:
    x[0] = 4
    print("FAIL")
    raise SystemExit
except TypeError:
    pass
print(x)
try:
    x.append(5)
    print("FAIL")
    raise SystemExit
except AttributeError:
    pass

print(x[1:])
print(x[:-1])
print(x[2:3])

print(x + (10, 100, 10000))

# inplace add operator
x += (10, 11, 12)
print(x)

# construction of tuple from large iterator (tests implementation detail of uPy)
print(tuple(range(20)))

# unsupported unary operation
try:
    +()
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

# unsupported type on RHS of add
try:
    () + None
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

print("PASS")