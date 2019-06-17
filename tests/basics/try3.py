# nested exceptions

passed = 1

def f():
    try:
        foo()
        passed = 0;
    except:
        print("except 1")
        try:
            baz()
            passed = 0;
        except:
            print("except 2")
        bar()

try:
    f()
    passed = 0;
except:
    print("f except")

if (passed):
    print("PASS")
else:
    print("FAIL")
