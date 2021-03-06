# case where generator doesn't intercept the thrown/injected exception
def gen():
    yield 123
    yield 456

g = gen()
print(next(g))
try:
    g.throw(KeyError)
except KeyError:
    pass

# case where a thrown exception is caught and stops the generator
def gen():
    try:
        yield 1
        yield 2
    except SystemExit:
        raise
    except:
        pass
g = gen()
print(next(g))
try:
    g.throw(ValueError)
except StopIteration:
    pass

# generator ignores a thrown GeneratorExit (this is allowed)
def gen():
    try:
        yield 123
    except GeneratorExit:
        pass
    yield 456

# thrown a class
g = gen()
print(next(g))
print(g.throw(GeneratorExit))

# thrown an instance
g = gen()
print(next(g))
print(g.throw(GeneratorExit()))

# thrown an instance with None as second arg
g = gen()
print(next(g))
print(g.throw(GeneratorExit()))

# thrown a class and instance
g = gen()
print(next(g))
print(g.throw(GeneratorExit, GeneratorExit(123)))

print("PASS")