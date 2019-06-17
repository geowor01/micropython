class C:
    def f():
        pass

# del a class attribute

del C.f
try:
    print(C.x)
    print("FAIL")
    raise SystemExit
except AttributeError:
    pass
try:
    del C.f
    print("FAIL")
    raise SystemExit
except AttributeError:
    pass

# del an instance attribute

c = C()

c.x = 1
print(c.x)

del c.x
try:
    print(c.x)
    print("FAIL")
    raise SystemExit
except AttributeError:
    pass
try:
    del c.x
    print("FAIL")
    raise SystemExit
except AttributeError:
    pass

# try to del an attribute of a built-in class
try:
    del int.to_bytes
    print("FAIL")
    raise SystemExit
except (AttributeError, TypeError):
    # uPy raises AttributeError, CPython raises TypeError
    print("PASS")
