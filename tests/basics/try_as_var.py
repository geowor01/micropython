try:
    raise ValueError(534)
    print("FAIL")
    raise SystemExit
except ValueError as e:
    pass

# Var bound in except block is automatically deleted
try:
    e
except NameError:
    print("PASS")
