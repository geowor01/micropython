# make sure type of first arg (self) to a builtin method is checked

list.append

try:
    list.append()
    print("FAIL")
    raise SystemExit
except TypeError as e:
    pass

try:
    list.append(1)
    print("FAIL")
    raise SystemExit
except TypeError as e:
    pass

try:
    list.append(1, 2)
    print("FAIL")
    raise SystemExit
except TypeError as e:
    pass

l = []
list.append(l, 2)
print(l)

try:
    getattr(list, "append")(1, 2)
    print("FAIL")
    raise SystemExit
except TypeError as e:
    pass

l = []
getattr(list, "append")(l, 2)
print(l)

print("PASS")