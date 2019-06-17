# delete local then try to reference it
def f():
    x = 1
    y = 2
    print(x, y)
    del x
    print(y)
    try:
        print(x)
        print("FAIL")
        raise SystemExit
    except NameError:
        pass
f()

# delete local then try to delete it again
def g():
    x = 3
    y = 4
    print(x, y)
    del x
    print(y)
    try:
        del x
        print("FAIL")
        raise SystemExit
    except NameError:
        pass
g()

print("PASS")