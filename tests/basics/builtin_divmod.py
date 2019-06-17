# test builtin divmod

print(divmod(0, 2))
print(divmod(3, 4))
print(divmod(20, 3))

try:
    divmod(1, 0)
    print("FAIL")
    raise SystemExit
except ZeroDivisionError:
    pass

try:
    divmod('a', 'b')
    print("FAIL")
    raise SystemExit
except TypeError:
    print("PASS")
