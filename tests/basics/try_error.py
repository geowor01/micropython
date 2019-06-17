# test bad exception match

try:
    try:
        a
    except 1:
        pass
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

try:
    try:
        a
    except (1,):
        pass
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

print("PASS")