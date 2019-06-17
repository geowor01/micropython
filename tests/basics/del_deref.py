def f():
    x = 1
    y = 2
    def g():
        nonlocal x
        print(y)
        try:
            print(x)
            print("FAIL")
            raise SystemExit
        except NameError:
            pass
    def h():
        nonlocal x
        print(y)
        try:
            del x
            print("FAIL")
            raise SystemExit
        except NameError:
            pass
    print(x, y)
    del x
    g()
    h()
f()

print("PASS")