# test use of return with try-except

def f():
    try:
        print(1)
        return
    except:
        print(2)
    print(3)
f()

def f(l, i):
    try:
        return l[i]
        print("FAIL")
        raise SystemExit
    except IndexError:
        pass
        return -1

print(f([1], 0))
print(f([], 0))

print("PASS")