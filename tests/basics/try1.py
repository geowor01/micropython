# basic exceptions
x = 1
try:
    x.a()
except:
    print(x)

try:
    raise IndexError
    print("FAIL")
    raise SystemExit
except IndexError:
    print("caught")

print("PASS")