def gen():
    try:
        yield 1
        print("FAIL")
        raise SystemExit
    except ValueError as e:
        pass
    yield "str1"
    raise TypeError

def gen2():
    print((yield from gen()))

g = gen2()
print(next(g))
print(g.throw(ValueError))
try:
    print(next(g))
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

# passing None as second argument to throw
g = gen2()
print(next(g))
print(g.throw(ValueError))
try:
    print(next(g))
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

# passing an exception instance as second argument to throw
g = gen2()
print(next(g))
print(g.throw(ValueError, ValueError(123)))
try:
    print(next(g))
    print("FAIL")
    raise SystemExit
except TypeError:
    print("PASS")
