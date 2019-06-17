# locals referenced before assignment

def f1():
    print(x)
    x = 1

def f2():
    for i in range(0):
        print(i)
    print(i)

def check(f):
    try:
        f()
        print("FAIL")
        raise SystemExit
    except NameError:
        pass

check(f1)
check(f2)

print("PASS")