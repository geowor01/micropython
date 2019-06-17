# test if eval raises SyntaxError

try:
    eval
except NameError:
    print("SKIP")
    raise SystemExit

try:
    print(eval("[1,,]"))
    print("FAIL")
    raise SystemExit
except SyntaxError:
    print("PASS")
