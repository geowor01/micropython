def gen():
    yield 1
    yield 2
    raise ValueError

def gen2():
    try:
        print((yield from gen()))
        print("FAIL")
        raise SystemExit
    except ValueError:
        pass

g = gen2()
print(list(g))

print("PASS")