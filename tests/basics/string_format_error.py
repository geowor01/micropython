# tests for errors in {} format string

try:
    '{0:0}'.format('zzz')
    print("FAIL")
    raise SystemExit
except (ValueError):
    pass

try:
    '{1:}'.format(1)
    print("FAIL")
    raise SystemExit
except IndexError as e:
    pass

try:
    '}'.format('zzzz')
    print("FAIL")
    raise SystemExit
except ValueError as e:
    pass

# end of format parsing conversion specifier
try:
    '{!'.format('a')
    print("FAIL")
    raise SystemExit
except ValueError as e:
    pass

# unknown conversion specifier
try:
    'abc{!d}'.format('1')
    print("FAIL")
    raise SystemExit
except ValueError as e:
    pass

try:
    '{abc'.format('zzzz')
    print("FAIL")
    raise SystemExit
except ValueError as e:
    pass

# expected ':' after specifier
try:
    '{!s :}'.format(2)
    print("FAIL")
    raise SystemExit
except ValueError as e:
    pass

try:
    '{}{0}'.format(1, 2)
    print("FAIL")
    raise SystemExit
except ValueError as e:
    pass

try:
    '{1:}'.format(1)
    print("FAIL")
    raise SystemExit
except IndexError as e:
    pass

try:
    '{ 0 :*^10}'.format(12)
    print("FAIL")
    raise SystemExit
except KeyError as e:
    pass

try:
    '{0}{}'.format(1)
    print("FAIL")
    raise SystemExit
except ValueError as e:
    pass

try:
    '{}{}'.format(1)
    print("FAIL")
    raise SystemExit
except IndexError as e:
    pass

try:
    '{0:+s}'.format('1')
    print("FAIL")
    raise SystemExit
except ValueError as e:
    pass

try:
    '{0:+c}'.format(1)
    print("FAIL")
    raise SystemExit
except ValueError as e:
    pass

try:
    '{0:s}'.format(1)
    print("FAIL")
    raise SystemExit
except ValueError as e:
    pass

try:
    '{:*"1"}'.format('zz')
    print("FAIL")
    raise SystemExit
except ValueError as e:
    pass

# unknown format code for str arg
try:
    '{:X}'.format('zz')
    print("FAIL")
    raise SystemExit
except ValueError as e:
    pass

print("PASS")