# test set binary operations

sets = [set(), {1}, {1, 2}, {1, 2, 3}, {2, 3}, {2, 3, 5}, {5}, {7}]
for s in sets:
    for t in sets:
        print(sorted(s), '|', sorted(t), '=', sorted(s | t))
        print(sorted(s), '^', sorted(t), '=', sorted(s ^ t))
        print(sorted(s), '&', sorted(t), '=', sorted(s & t))
        print(sorted(s), '-', sorted(t), '=', sorted(s - t))
        u = s.copy()
        u |= t
        print(sorted(s), "|=", sorted(t), '-->', sorted(u))
        u = s.copy()
        u ^= t
        print(sorted(s), "^=", sorted(t), '-->', sorted(u))
        u = s.copy()
        u &= t
        print(sorted(s), "&=", sorted(t), "-->", sorted(u))
        u = s.copy()
        u -= t
        print(sorted(s), "-=", sorted(t), "-->", sorted(u))

        print(sorted(s), '==', sorted(t), '=', s == t)
        print(sorted(s), '!=', sorted(t), '=', s != t)
        print(sorted(s), '>', sorted(t), '=', s > t)
        print(sorted(s), '>=', sorted(t), '=', s >= t)
        print(sorted(s), '<', sorted(t), '=', s < t)
        print(sorted(s), '<=', sorted(t), '=', s <= t)

print(set('abc') == 1)

# make sure inplace operators modify the set

s1 = s2 = set('abc')
s1 |= set('ad')
print(s1 is s2, len(s1))

s1 = s2 = set('abc')
s1 ^= set('ad')
print(s1 is s2, len(s1))

s1 = s2 = set('abc')
s1 &= set('ad')
print(s1 is s2, len(s1))

s1 = s2 = set('abc')
s1 -= set('ad')
print(s1 is s2, len(s1))

print(set('12') >= '1')
print(set('12') <= '123')

# unsupported operator
try:
    set('abc') * set('abc')
    print("FAIL")
    raise SystemExit
except TypeError as e:
    pass

# unsupported operator with RHS not a set
try:
    set('abc') * 2
    print("FAIL")
    raise SystemExit
except TypeError as e:
    pass

print("PASS")