# test for+range, mostly to check optimisation of this pair

# apply args using *
for x in range(*(1, 3)):
    print(x)
for x in range(1, *(6, 2)):
    print(x)

# zero step
try:
    for x in range(1, 2, 0):
        pass
    print("FAIL")
    raise SystemExit
except ValueError:
    pass

# apply args using **
try:
    for x in range(**{'end':1}):
        print(x)
    print("FAIL")
    raise SystemExit
except TypeError:
    pass
try:
    for x in range(0, **{'end':1}):
        print(x)
    print("FAIL")
    raise SystemExit
except TypeError:
    pass
try:
    for x in range(0, 1, **{'step':1}):
        print(x)
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

# keyword args
try:
    for x in range(end=1):
        print(x)
    print("FAIL")
    raise SystemExit
except TypeError:
    pass
try:
    for x in range(0, end=1):
        print(x)
    print("FAIL")
    raise SystemExit
except TypeError:
    pass
try:
    for x in range(start=0, end=1):
        print(x)
    print("FAIL")
    raise SystemExit
except TypeError:
    pass
try:
    for x in range(0, 1, step=1):
        print(x)
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

# argument is a comprehension
try:
    for x in range(0 for i in []):
        print(x)
    print("FAIL")
    raise SystemExit
except TypeError:
    pass
try:
    for x in range(0, (0 for i in [])):
        print(x)
    print("FAIL")
    raise SystemExit
except TypeError:
    pass
try:
    for x in range(0, 1, (0 for i in [])):
        print(x)
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

print("PASS")