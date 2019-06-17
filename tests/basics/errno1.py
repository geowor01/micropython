# test errno's and uerrno module

try:
    import uerrno
except ImportError:
    print("SKIP")
    raise SystemExit

# check that constants exist and are integers
if (str(type(uerrno.EIO)) != "<class 'int'>"):
    print("FAIL")
    raise SystemExit

# check that errors are rendered in a nice way
msg = str(OSError(uerrno.EIO))
print(msg[:7], msg[-5:])

# check that unknown errno is still rendered
if (str(OSError(9999)) != 9999):
    print("FAIL")
    raise SystemExit

# this tests a failed constant lookup in errno
errno = uerrno
if (str(errno.__name__) != "uerrno"):
    print("FAIL")
    raise SystemExit

print("PASS")