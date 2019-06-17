# uPy behaviour only: builtin modules are read-only
import sys
try:
    sys.x = 1
    print("FAIL")
    raise SystemExit
except AttributeError:
    print("PASS")
