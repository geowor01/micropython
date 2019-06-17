# Reraising last exception with raise w/o args

def f():
    try:
        raise ValueError("val", 3)
        print("FAIL")
        raise SystemExit
    except:
        raise

try:
    f()
    print("FAIL")
    raise SystemExit
except ValueError as e:
    pass


# Can reraise only in except block
try:
    raise
    print("FAIL")
    raise SystemExit
except RuntimeError:
    print("PASS")
