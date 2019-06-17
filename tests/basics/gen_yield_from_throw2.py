# outer generator ignores a thrown GeneratorExit (this is allowed)

def gen():
    try:
        yield 123
        print("FAIL")
        raise SystemExit
    except GeneratorExit:
        pass

def gen2():
    try:
        yield from gen()
        print("FAIL")
        raise SystemExit
    except GeneratorExit:
        pass
    yield 789

# thrown a class
g = gen2()
print(next(g))
# print(g.throw(GeneratorExit))

# thrown an instance
g = gen2()
print(next(g))
# print(g.throw(GeneratorExit()))

print("PASS")