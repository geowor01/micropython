a = [1, 2, 3]
print(a.index(1))
print(a.index(2))
print(a.index(3))
print(a.index(3, 2))
print(a.index(1, -100))
print(a.index(1, False))

try:
    print(a.index(1, True))
    print("FAIL")
    raise SystemExit
except ValueError:
    pass

try:
    print(a.index(3, 2, 2))
    print("FAIL")
    raise SystemExit
except ValueError:
    pass

a = a + a
b = [0, 0, a]
print(a.index(2))
print(b.index(a))
print(a.index(2, 2))

try:
    a.index(2, 2, 2)
    print("FAIL")
    raise SystemExit
except ValueError:
    pass

# 3rd argument to index greater than length of list
print([1, 2].index(1, 0, 4))

print("PASS")