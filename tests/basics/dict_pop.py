d = {1: 2, 3: 4}
print(d.pop(3), d)
print(d)
print(d.pop(1, 42), d)
print(d.pop(1, 42), d)
print(d.pop(1, None), d)
try:
    print(d.pop(1), "!!!",)
    print("FAIL")
    raise SystemExit
except KeyError:
    print("PASS")

print("PASS")