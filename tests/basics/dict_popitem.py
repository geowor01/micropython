els = []
d = {1:2,3:4}
a = d.popitem()
print(len(d))
els.append(a)
a = d.popitem()
print(len(d))
els.append(a)
try:
    print(d.popitem(), "!!!",)
    print("FAIL")
    raise SystemExit
except KeyError:
    pass
print(sorted(els))

print("PASS")