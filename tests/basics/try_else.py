# test try-else statement
passed = 1
# base case
try:
    print(1)
except:
    passed = 0
    print(2)
else:
    print(3)

# basic case that should skip else
try:
    print(1)
    raise Exception
    passed = 0
except:
    print(2)
else:
    passed = 0
    print(3)

# uncaught exception should skip else
try:
    try:
        print(1)
        raise ValueError
        passed = 0
    except TypeError:
        print(2)
    else:
        passed = 0
        print(3)
except:
    print('caught')

# nested within outer try
try:
    print(1)
    try:
        print(2)
        raise Exception
        passed = 0
    except:
        print(3)
    else:
        passed = 0
        print(4)
except:
    passed = 0
    print(5)
else:
    print(6)

# nested within outer except, one else should be skipped
try:
    print(1)
    raise Exception
    passed = 0
except:
    print(2)
    try:
        print(3)
    except:
        passed = 0
        print(4)
    else:
        print(5)
else:
    passed = 0
    print(6)

# nested within outer except, both else should be skipped
try:
    print(1)
    raise Exception
    passed = 0
except:
    print(2)
    try:
        print(3)
        raise Exception
        passed = 0
    except:
        print(4)
    else:
        passed = 0
        print(5)
else:
    passed = 0
    print(6)

if (passed):
    print("PASS")
else:
    print("FAIL")