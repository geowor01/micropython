# del name

x = 1
print(x)
del x
try:
    print(x)
    print("FAIL")
    raise SystemExit
except NameError:
    pass
try:
    del x
    print("FAIL")
    raise SystemExit
except: # NameError:
    # FIXME uPy returns KeyError for this
    pass

class C:
    def f():
        pass

print("PASS")