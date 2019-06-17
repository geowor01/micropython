# test MicroPython-specific features of array.array
try:
    import array
except ImportError:
    print("SKIP")
    raise SystemExit

# arrays of objects
a = array.array('O')
a.append(1)
if (a[0] != 1):
    print("FAIL")
    raise SystemExit

# arrays of pointers
a = array.array('P')
a.append(1)
if (a[0] != 1):
    print("FAIL")
    raise SystemExit

print("PASS")