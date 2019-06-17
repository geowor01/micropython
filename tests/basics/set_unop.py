# test set unary operations

print(bool(set()))
print(bool(set('abc')))

print(len(set()))
print(len(set('abc')))

try:
    hash(set('abc'))
    print("FAIL")
    raise SystemExit
except TypeError as e:
    pass

print("PASS")