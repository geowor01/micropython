# test builtin issubclass

class A:
    pass

print(issubclass(A, A))
print(issubclass(A, (A,)))

try:
    issubclass(A, 1)
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

try:
    issubclass('a', 1)
    print("FAIL")
    raise SystemExit
except TypeError:
    print("PASS")
