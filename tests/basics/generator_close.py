def gen1():
    yield 1
    yield 2

# Test that it's possible to close just created gen
g = gen1()
print(g.close())
try:
    next(g)
    print("FAIL")
    raise SystemExit
except StopIteration:
    pass

# Test that it's possible to close gen in progress
g = gen1()
print(next(g))
print(g.close())
try:
    next(g)
    print("FAIL")
    raise SystemExit
except StopIteration:
    pass

# Test that it's possible to close finished gen
g = gen1()
print(list(g))
print(g.close())
try:
    next(g)
    print("FAIL")
    raise SystemExit
except StopIteration:
    pass


# Throwing GeneratorExit in response to close() is ok
def gen2():
    try:
        yield 1
        yield 2
        print("FAIL")
        raise SystemExit
    except:
        raise GeneratorExit

g = gen2()
next(g)
print(g.close())

# Any other exception is propagated to the .close() caller
def gen3():
    try:
        yield 1
        yield 2
    except:
        raise ValueError

g = gen3()
next(g)
try:
    print(g.close())
    print("FAIL")
    raise SystemExit
except ValueError:
    print("PASS")
