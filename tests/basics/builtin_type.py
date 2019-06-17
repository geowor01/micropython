# test builtin type

print(type(int))

try:
    type()
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

try:
    type(1, 2)
    print("FAIL")
    raise SystemExit
except TypeError:
    pass

# # second arg should be a tuple
# try:
#     type('abc', None, None)
#     print("FAIL")
#     raise SystemExit
# except TypeError:
#     pass

# # third arg should be a dict
# try:
#     type('abc', (), None)
#     print("FAIL")
#     raise SystemExit
# except TypeError:
#     pass

# # elements of second arg (the bases) should be types
# try:
#     type('abc', (1,), {})
#     print("FAIL")
#     raise SystemExit
# except TypeError:
#     print("PASS")

print("PASS")