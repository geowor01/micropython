class A:

    var = 132

    def __init__(self):
        self.var2 = 34


a = A()
setattr(a, "var", 123)
setattr(a, "var2", 56)
print(a.var)
print(a.var2)

try:
    setattr(a, b'var3', 1)
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

# try setattr on a built-in function
try:
    setattr(int, 'to_bytes', 1)
    print("FAIL")
    raise SystemExit
except (AttributeError, TypeError):
    print("PASS")
