# test errors from bad operations (unary, binary, etc)

def test_exc(code, exc):
    try:
        exec(code)
        print("FAIL")
        raise SystemExit
    except exc:
        pass

# object with buffer protocol needed on rhs
try:
    (1 << 70) in 1
    print("FAIL")
    raise SystemExit
except TypeError:
    print("PASS")
