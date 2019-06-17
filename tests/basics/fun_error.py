# test errors from bad function calls

# function doesn't take keyword args
try:
    [].append(x=1)
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

# function with variable number of positional args given too few
try:
    round()
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

# function with variable number of positional args given too many
try:
    round(1, 2, 3)
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

# function with fixed number of positional args given wrong number
try:
    [].append(1, 2)
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

# function with keyword args given extra positional args
try:
    [].sort(1)
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

# function with keyword args given extra keyword args
try:
    [].sort(noexist=1)
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

# kw given for positional, but a different positional is missing
try:
    def f(x, y): pass
    f(x=1)
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

print("PASS")