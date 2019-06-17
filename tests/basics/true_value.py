# Test true-ish value handling

if not False:
    print("False")
else:
    print("FAIL")
    raise SystemExit

if not None:
    print("None")
else:
    print("FAIL")
    raise SystemExit

if not 0:
    print("0")
else:
    print("FAIL")
    raise SystemExit

if not "":
    print("Empty string")
else:
    print("FAIL")
    raise SystemExit
if "foo":
    print("Non-empty string")
else:
    print("FAIL")
    raise SystemExit

if not ():
    print("Empty tuple")
else:
    print("FAIL")
    raise SystemExit
if ("",):
    print("Non-empty tuple")
else:
    print("FAIL")
    raise SystemExit

if not []:
    print("Empty list")
else:
    print("FAIL")
    raise SystemExit
if [0]:
    print("Non-empty list")
else:
    print("FAIL")
    raise SystemExit

if not {}:
    print("Empty dict")
else:
    print("FAIL")
    raise SystemExit
if {0:0}:
    print("Non-empty dict")
else:
    print("FAIL")
    raise SystemExit

print("PASS")