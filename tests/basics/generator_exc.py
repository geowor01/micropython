# Test proper handling of exceptions within generator across yield
def gen():
    try:
        yield 1
        raise ValueError
        print("FAIL")
        raise SystemExit
    except ValueError:
        pass
    yield 2

for i in gen():
    print(i)


# Test throwing exceptions out of generator
def gen2():
    yield 1
    raise ValueError
    yield 2
    yield 3

g = gen2()
print(next(g))
try:
    print(next(g))
    print("FAIL")
    raise SystemExit
except ValueError:
    pass

try:
    print(next(g))
    print("FAIL")
    raise SystemExit
except StopIteration:
    pass


# Test throwing exception into generator
def gen3():
    yield 1
    try:
        yield 2
        print("FAIL")
        raise SystemExit
    except ValueError:
        yield 3
    yield 4
    yield 5

g = gen3()
print(next(g))
print(next(g))
print("out of throw:", g.throw(ValueError))
print(next(g))
try:
    print("out of throw2:", g.throw(ValueError))
    print("FAIL")
    raise SystemExit
except ValueError:
    print("PASS")