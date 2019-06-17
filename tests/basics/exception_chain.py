# Exception chaining is not supported, but check that basic
# exception works as expected.
try:
    raise Exception from None
    print("FAIL")
    raise SystemExit
except Exception:
    print("PASS")
