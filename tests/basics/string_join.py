print(','.join(()))
print(','.join(('a',)))
print(','.join(('a', 'b')))

print(','.join([]))
print(','.join(['a']))
print(','.join(['a', 'b']))

print(''.join(''))
print(''.join('abc'))
print(','.join('abc'))
print(','.join('abc' for i in range(5)))

print(b','.join([b'abc', b'123']))

try:
    ''.join(None)
    print("FAIL")
    raise SystemExit
except TypeError as e:
    pass

try:
    print(b','.join(['abc', b'123']))
    print("FAIL")
    raise SystemExit
except TypeError as e:
    pass

try:
    print(','.join([b'abc', b'123']))
    print("FAIL")
    raise SystemExit
except TypeError as e:
    pass

# joined by the compiler
print("a" "b")
print("a" '''b''')
print("a" # inline comment
    "b")
print("a" \
    "b")

# the following should not be joined by the compiler
x = 'a'
'b'
print(x)

print("PASS")