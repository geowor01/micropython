# nested try's

passed = 1

try:
    print("try 1")
    try:
        print("try 2")
        foo()
        passed = 0
    except:
        print("except 2")
    bar()
    passed = 0
except:
    print("except 1")

try:
    print("try 1")
    try:
        print("try 2")
        foo()
        passed = 0
    except TypeError:
        print("except 2")
    bar()
    passed = 0
except NameError:
    print("except 1")

# raised exception not contained in except tuple
try:
    try:
        raise Exception
        passed = 0
    except (RuntimeError, SyntaxError):
        print('except 2')
except Exception:
    print('except 1')

# Check that exceptions across function boundaries work as expected
def func1():
    try:
        print("try func1")
        func2()
        passed = 0
    except NameError:
        print("except func1")

def func2():
    try:
        print("try func2")
        foo()
        passed = 0
    except TypeError:
        print("except func2")

func1()

if (passed):
    print("PASS")
else:
    print("FAIL")
