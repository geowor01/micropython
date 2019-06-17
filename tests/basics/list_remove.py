a = [1, 2, 3]
print(a.remove(2))
print(a)
try:
    a.remove(2)
    print("FAIL")
    raise SystemExit
except ValueError:
    print("PASS")
