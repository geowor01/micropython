# tests for correct PEP479 behaviour (introduced in Python 3.5)

# basic case: StopIteration is converted into a RuntimeError
def gen():
    yield 1
    raise StopIteration
g = gen()

if (next(g) != 1):
    print("FAIL")
    raise SystemExit

try:
    next(g)
    print("FAIL")
    raise SystemExit
except StopIteration:
    pass

# trying to continue a failed generator now raises StopIteration
try:
    next(g)
    print("FAIL")
    raise SystemExit
except StopIteration:
    pass

# throwing a StopIteration which is uncaught will be converted into a RuntimeError
def gen():
    yield 1
    yield 2
g = gen()
if (next(g) != 1):
    print("FAIL")
    raise SystemExit
try:
    g.throw(StopIteration)
    print("FAIL")
    raise SystemExit
except StopIteration:
    print("PASS")
