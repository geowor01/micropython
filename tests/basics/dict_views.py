d = {1: 2}
for m in d.items, d.values, d.keys:
    print(m())
    print(list(m()))

# print a view with more than one item
print({1:1, 2:1}.values())

# unsupported binary op on a dict values view
try:
    {1:1}.values() + 1
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

# unsupported binary op on a dict keys view
try:
    {1:1}.keys() + 1
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

# set operations still to come
print("PASS")