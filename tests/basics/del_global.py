# del global

def do_del():
    global x
    del x

x = 1
print(x)
do_del()
try:
    print(x)
    print("FAIL")
    raise SystemExit
except NameError:
    pass
try:
    do_del()
    print("FAIL")
    raise SystemExit
except: # NameError:
    # FIXME uPy returns KeyError for this
    pass

# delete globals using a list

a = 1
del (a,)
try:
    print(a)
    print("FAIL")
    raise SystemExit
except NameError:
    pass

a = 2
b = 3
del (a, b)
try:
    print(a)
    print("FAIL")
    raise SystemExit
except NameError:
    pass
try:
    print(b)
    print("FAIL")
    raise SystemExit
except NameError:
    pass

a = 1
b = 2
c = 3
del (a, b, c)
try:
    print(a)
    print("FAIL")
    raise SystemExit
except NameError:
    pass
try:
    print(b)
    print("FAIL")
    raise SystemExit
except NameError:
    pass
try:
    print(c)
    print("FAIL")
    raise SystemExit
except NameError:
    pass

a = 1
b = 2
c = 3
del (a, (b, c))

print("PASS")