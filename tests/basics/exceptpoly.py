try:
    raise ArithmeticError
    print("FAIL")
    raise SystemExit
except Exception:
    print("Caught ArithmeticError via Exception")

try:
    raise ArithmeticError
    print("FAIL")
    raise SystemExit
except ArithmeticError:
    print("Caught ArithmeticError")

try:
    raise AssertionError
    print("FAIL")
    raise SystemExit
except Exception:
    print("Caught AssertionError via Exception")

try:
    raise AssertionError
    print("FAIL")
    raise SystemExit
except AssertionError:
    print("Caught AssertionError")

try:
    raise AttributeError
    print("FAIL")
    raise SystemExit
except Exception:
    print("Caught AttributeError via Exception")

try:
    raise AttributeError
    print("FAIL")
    raise SystemExit
except AttributeError:
    print("Caught AttributeError")

try:
    raise EOFError
    print("FAIL")
    raise SystemExit
except Exception:
    print("Caught EOFError via Exception")

try:
    raise EOFError
    print("FAIL")
    raise SystemExit
except EOFError:
    print("Caught EOFError")

try:
    raise Exception
    print("FAIL")
    raise SystemExit
except BaseException:
    print("Caught Exception via BaseException")

try:
    raise Exception
    print("FAIL")
    raise SystemExit
except Exception:
    print("Caught Exception")

try:
    raise ImportError
    print("FAIL")
    raise SystemExit
except Exception:
    print("Caught ImportError via Exception")

try:
    raise ImportError
    print("FAIL")
    raise SystemExit
except ImportError:
    print("Caught ImportError")

try:
    raise IndentationError
    print("FAIL")
    raise SystemExit
except SyntaxError:
    print("Caught IndentationError via SyntaxError")

try:
    raise IndentationError
    print("FAIL")
    raise SystemExit
except IndentationError:
    print("Caught IndentationError")

try:
    raise IndexError
    print("FAIL")
    raise SystemExit
except LookupError:
    print("Caught IndexError via LookupError")

try:
    raise IndexError
    print("FAIL")
    raise SystemExit
except IndexError:
    print("Caught IndexError")

try:
    raise KeyError
    print("FAIL")
    raise SystemExit
except LookupError:
    print("Caught KeyError via LookupError")

try:
    raise KeyError
    print("FAIL")
    raise SystemExit
except KeyError:
    print("Caught KeyError")

try:
    raise LookupError
    print("FAIL")
    raise SystemExit
except Exception:
    print("Caught LookupError via Exception")

try:
    raise LookupError
    print("FAIL")
    raise SystemExit
except LookupError:
    print("Caught LookupError")

print("PASS")