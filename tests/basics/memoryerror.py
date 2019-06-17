# test out-of-memory with malloc
l = list(range(1000))
try:
    1000000000 * l
    print("FAIL")
    raise SystemExit
except MemoryError:
    pass
if (len(l) != 1000 or l[0] != 0 or l[-1] != 999):
    print("FAIL")
    raise SystemExit

# test out-of-memory with realloc
try:
    [].extend(range(1000000000))
    print("FAIL")
    raise SystemExit
except MemoryError:
    print("PASS")
