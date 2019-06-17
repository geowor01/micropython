try:
    try:
        from ucollections import namedtuple
    except ImportError:
        from collections import namedtuple
except ImportError:
    print("SKIP")
    raise SystemExit

T = namedtuple("Tup", ["foo", "bar"])
# CPython prints fully qualified name, what we don't bother to do so far
#print(T)
for t in T(1, 2), T(bar=1, foo=2):
    print(t)
    print(t[0], t[1])
    print(t.foo, t.bar)

    print(len(t))
    print(bool(t))
    print(t + t)
    print(t * 3)

    print([f for f in t])

    print(isinstance(t, tuple))

# Create using positional and keyword args
print(T(3, bar=4))

try:
    t[0] = 200
    print("FAIL")
    raise SystemExit
except TypeError:
    pass
try:
    t.bar = 200
    print("FAIL")
    raise SystemExit
except AttributeError:
    pass

try:
    t = T(1)
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

try:
    t = T(1, 2, 3)
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

try:
    t = T(foo=1)
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

try:
    t = T(1, foo=1)
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

# enough args, but kw is wrong
try:
    t = T(1, baz=3)
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

# bad argument for member spec
try:
    namedtuple('T', 1)
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

# Try tuple
T4 = namedtuple("TupTuple", ("foo", "bar"))
t = T4(1, 2)
print(t.foo, t.bar)

# Try single string with comma field separator
# Not implemented so far
#T2 = namedtuple("TupComma", "foo,bar")
#t = T2(1, 2)

print("PASS")