# test continue within exception handler

def f():
    lst = [1, 2, 3]
    for x in lst:
        print('a', x)
        try:
            if x == 2:
                raise Exception
                print("FAIL")
                raise SystemExit
        except Exception:
            continue
        print('b', x)
f()

print("PASS")