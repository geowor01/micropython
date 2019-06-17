print(dict([(1, "foo")]))
d = dict([("foo", "foo2"), ("bar", "baz")])
print(sorted(d.keys()))
print(sorted(d.values()))

try:
    dict(((1,),))
    print("FAIL")
    raise SystemExit
except ValueError:
    pass

try:
    dict(((1, 2, 3),))
    print("FAIL")
    raise SystemExit
except ValueError:
    pass

print("PASS")