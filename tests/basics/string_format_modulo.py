print("%%" % ())
print("=%s=" % 1)
print("=%s=%s=" % (1, 2))
print("=%s=" % (1,))
print("=%s=" % [1, 2])

print("=%s=" % "str")
print("=%r=" % "str")

try:
    print("=%s=%s=" % 1)
    print("FAIL")
    raise SystemExit
except TypeError as e:
    pass

try:
    print("=%s=%s=%s=" % (1, 2))
    print("FAIL")
    raise SystemExit
except TypeError as e:
    pass

try:
    print("=%s=" % (1, 2))
    print("FAIL")
    raise SystemExit
except TypeError as e:
    pass

print("%s" % True)
print("%s" % 1)
print("%.1s" % "ab")

print("%r" % True)
print("%r" % 1)

print("%c" % 48)
print("%c" % 'a')
print("%10s" % 'abc')
print("%-10s" % 'abc')

# Should be able to print dicts; in this case they aren't used
# to lookup keywords in formats like %(foo)s
print('%s' % {})
print('%s' % ({},))

# Cases when "*" used and there's not enough values total
try:
    print("%*s" % 5)
    print("FAIL")
    raise SystemExit
except TypeError as e:
    pass
try:
    print("%*.*s" % (1, 15))
    print("FAIL")
    raise SystemExit
except TypeError as e:
    pass

print("%(foo)s" % {"foo": "bar", "baz": False})
print("%s %(foo)s %(foo)s" % {"foo": 1})
try:
    print("%(foo)s" % {})
    print("FAIL")
    raise SystemExit
except KeyError:
    pass
# Using in "*" with dict got to fail
try:
    print("%(foo)*s" % {"foo": "bar"})
    print("FAIL")
    raise SystemExit
except TypeError as e:
    pass

# When using %(foo)s format the single argument must be a dict
try:
    '%(foo)s' % 1
    print("FAIL")
    raise SystemExit
except TypeError as e:
    pass
try:
    '%(foo)s' % ({},)
    print("FAIL")
    raise SystemExit
except TypeError as e:
    pass

try:
    '%(a' % {'a':1}
    print("FAIL")
    raise SystemExit
except ValueError:
    pass

try:
    '%.*d %.*d' % (20, 5)
    print("FAIL")
    raise SystemExit
except TypeError as e:
    pass

try:
    a = '%*' % 1
    print("FAIL")
    raise SystemExit
except (ValueError):
    pass

try:
    '%c' % 'aa'
    print("FAIL")
    raise SystemExit
except TypeError as e:
    pass

try:
    '%l' % 1
    print("FAIL")
    raise SystemExit
except ValueError:
    pass

try:
    'a%' % 1
    print("FAIL")
    raise SystemExit
except ValueError:
    print("PASS")
