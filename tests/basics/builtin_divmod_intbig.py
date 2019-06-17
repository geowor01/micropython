# test builtin divmod

try:
    divmod(1 << 65, 0)
    print("FAIL")
    raise SystemExit
except ZeroDivisionError:
    pass

# bignum
l = (1 << 65) + 123
print(divmod(3, l))
print(divmod(l, 5))
print(divmod(l + 3, l))
print(divmod(l * 20, l + 2))

print("PASS")