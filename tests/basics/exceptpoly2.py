passed = 1

try:
    raise MemoryError
    passed = 0
except Exception:
    print("Caught MemoryError via Exception")

try:
    raise MemoryError
    passed = 0
except MemoryError:
    print("Caught MemoryError")

try:
    raise NameError
    passed = 0
except Exception:
    print("Caught NameError via Exception")

try:
    raise NameError
    passed = 0
except NameError:
    print("Caught NameError")

try:
    raise NotImplementedError
    passed = 0
except RuntimeError:
    print("Caught NotImplementedError via RuntimeError")

try:
    raise NotImplementedError
    passed = 0
except NotImplementedError:
    print("Caught NotImplementedError")

try:
    raise OSError
    passed = 0
except Exception:
    print("Caught OSError via Exception")

try:
    raise OSError
    passed = 0
except OSError:
    print("Caught OSError")

try:
    raise OverflowError
    passed = 0
except ArithmeticError:
    print("Caught OverflowError via ArithmeticError")

try:
    raise OverflowError
    passed = 0
except OverflowError:
    print("Caught OverflowError")

try:
    raise RuntimeError
    passed = 0
except Exception:
    print("Caught RuntimeError via Exception")

try:
    raise RuntimeError
    passed = 0
except RuntimeError:
    print("Caught RuntimeError")

try:
    raise SyntaxError
    passed = 0
except Exception:
    print("Caught SyntaxError via Exception")

try:
    raise SyntaxError
    passed = 0
except SyntaxError:
    print("Caught SyntaxError")

try:
    raise TypeError
    passed = 0
except Exception:
    print("Caught TypeError via Exception")

try:
    raise TypeError
    passed = 0
except TypeError:
    print("Caught TypeError")

try:
    raise ValueError
    passed = 0
except Exception:
    print("Caught ValueError via Exception")

try:
    raise ValueError
    passed = 0
except ValueError:
    print("Caught ValueError")

try:
    raise ZeroDivisionError
    passed = 0
except ArithmeticError:
    print("Caught ZeroDivisionError via ArithmeticError")

try:
    raise ZeroDivisionError
    passed = 0
except ZeroDivisionError:
    print("Caught ZeroDivisionError")

if (passed):
    print("PASS")
else:
    print("FAIL")