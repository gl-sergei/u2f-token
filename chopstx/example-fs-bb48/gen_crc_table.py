# 
# Polynomial for CRC32:
#    x26 + x23 + x22 + x16 + x12 + x11 + x10 + x8 + x7 + x5 + x4 + x2 + x + 1
#
# When it is represented in binary, it's:
#    0x04C11DB7
#    =
#    0000 0100 1100 0001 0001 1101 1011 0111
# 
# When we put in reverse bit-order, it's
#    0xedb88320

for i in range(0,256):
    c = i
    for j in range(0,8):
        if (c&1):
            c = 0xEDB88320 ^ (c >> 1)
        else:
            c = c >> 1
    print("0x%08x," % c),
    if (i % 6) == 5:
        print("")
