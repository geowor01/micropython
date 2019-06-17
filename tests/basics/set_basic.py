# basic sets

s = {1}
print(s)

s = {3, 4, 3, 1}
print(sorted(s))

# expression in constructor
s = {1 + len(s)}
print(s)

# Sets are not hashable
try:
    {s: 1}
    print("FAIL")
    raise SystemExit
except TypeError as e:
    pass

print("PASS")