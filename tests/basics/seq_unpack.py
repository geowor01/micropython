# Basics
a, b = 1, 2
print(a, b)
a, b = (1, 2)
print(a, b)
(a, b) = 1, 2
print(a, b)
(a, b) = (1, 2)
print(a, b)

# Tuples/lists are optimized
a, b = [1, 2]
print(a, b)
[a, b] = 100, 200
print(a, b)

# optimised 3-way swap
a = 1
b = 2
c = 3
a, b, c = b, c, a
print(a, b, c)

try:
    a, b, c = (1, 2)
    print("FAIL")
    raise SystemExit
except ValueError as e:
    pass
try:
    a, b, c = [1, 2, 3, 4]
    print("FAIL")
    raise SystemExit
except ValueError as e:
    pass

# Generic iterable object
a, b, c = range(3)
print(a, b, c)
try:
    a, b, c = range(2)
    print("FAIL")
    raise SystemExit
except ValueError as e:
    pass
try:
    a, b, c = range(4)
    print("FAIL")
    raise SystemExit
except ValueError as e:
    pass

print("PASS")