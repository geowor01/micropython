try:
    1 // 0
    print("FAIL")
    raise SystemExit
except ZeroDivisionError:
    pass

try:
    1 % 0
    print("FAIL")
    raise SystemExit
except ZeroDivisionError:
    print("PASS")
