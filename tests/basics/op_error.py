# test errors from bad operations (unary, binary, etc)

# unsupported unary operators
try:
    ~None
    print("FAIL")
    raise SystemExit
except TypeError:
    pass
try:
    ~''
    print("FAIL")
    raise SystemExit
except TypeError:
    pass
try:
    ~[]
    print("FAIL")
    raise SystemExit
except TypeError:
    pass
try:
    ~bytearray()
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

# unsupported binary operators
try:
    False in True
    print("FAIL")
    raise SystemExit
except TypeError:
    pass
try:
    1 * {}
    print("FAIL")
    raise SystemExit
except TypeError:
    pass
try:
    1 in 1
    print("FAIL")
    raise SystemExit
except TypeError:
    pass
try:
    bytearray() // 2
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

# object with buffer protocol needed on rhs
try:
    bytearray(1) + 1
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

# unsupported subscription
try:
    1[0]
    print("FAIL")
    raise SystemExit
except TypeError:
    pass
try:
    1[0] = 1
    print("FAIL")
    raise SystemExit
except TypeError:
    pass
try:
    ''['']
    print("FAIL")
    raise SystemExit
except TypeError:
    pass
try:
    'a'[0] = 1
    print("FAIL")
    raise SystemExit
except TypeError:
    pass
try:
    del 1[0]
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

# not callable
try:
    1()
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

# not an iterator
try:
    next(1)
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

# must be an exception type
try:
    raise 1
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

# no such name in import
try:
    from sys import youcannotimportmebecauseidontexist
    print("FAIL")
    raise SystemExit
except ImportError:
    print("PASS")
