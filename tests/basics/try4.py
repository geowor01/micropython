# triple nested exceptions

passed = 1

def f():
    try:
        foo()
        passed = 0
    except:
        print("except 1")
        try:
            bar()
            passed = 0
        except:
            print("except 2")
            try:
                baz()
                passed = 0
            except:
                print("except 3")
            bak()
            passed = 0

try:
    f()
    passed = 0
except:
    print("f except")

if (passed):
    print("PASS")
else:
    print("FAIL")
