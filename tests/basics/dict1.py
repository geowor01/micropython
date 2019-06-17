# basic dictionary

d = {}
print(d)
d[2] = 123
print(d)
d = {1:2}
d[3] = 3
print(len(d), d[1], d[3])
d[1] = 0
print(len(d), d[1], d[3])
print(str(d) == '{1: 0, 3: 3}' or str(d) == '{3: 3, 1: 0}')

x = 1
while x < 100:
    d[x] = x
    x += 1
print(d[50])

# equality operator on dicts of different size
print({} == {1:1})

# equality operator on dicts of same size but with different keys
print({1:1} == {2:1})

# value not found
try:
    {}[0]
    print("FAIL")
    raise SystemExit
except KeyError as er:
    pass

# unsupported unary op
try:
    +{}
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

# unsupported binary op
try:
    {} + {}
    print("FAIL")
    raise SystemExit
except TypeError:
    print("PASS")
