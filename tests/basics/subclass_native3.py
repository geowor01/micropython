class MyExc(Exception):
    pass

e = MyExc(100, "Some error")
print(e)
print(repr(e))
print(e.args)

try:
    raise MyExc("Some error", 1)
    print("FAIL")
    raise SystemExit
except MyExc as e:
    pass

try:
    raise MyExc("Some error2", 2)
    print("FAIL")
    raise SystemExit
except Exception as e:
    pass

try:
    raise MyExc("Some error2")
    print("FAIL")
    raise SystemExit
except:
    pass

print("PASS")