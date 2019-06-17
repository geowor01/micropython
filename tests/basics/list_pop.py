# list poppin'
a = [1, 2, 3]
print(a.pop())
print(a.pop())
print(a.pop())
try:
    print(a.pop())
    print("FAIL")
    raise SystemExit
except IndexError:
    pass

# popping such that list storage shrinks (tests implementation detail of uPy)
l = list(range(20))
for i in range(len(l)):
    l.pop()
print(l)

print("PASS")