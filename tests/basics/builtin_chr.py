# test builtin chr (whether or not we support unicode)

print(chr(65))

try:
    chr(0x110000)
    print("FAIL")
except ValueError:
    print("PASS")

