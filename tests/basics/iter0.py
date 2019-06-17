# builtin type that is not iterable
try:
    for i in 1:
        pass
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

# builtin type that is iterable, calling __next__ explicitly
print(iter(range(4)).__next__())

print("PASS")