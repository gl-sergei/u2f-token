#!/usr/bin/env python

#
# Use this script to inject your own private key and authentication counter
# into U2F binary. Might be useful if you want keys to survive firmware updates.
#
# Example:
#
# Generate EC private key with openssl:
# > openssl ecparam -name prime256v1 -genkey -noout -outform der > key.der
#
# Inject generated key into u2f.bin and set auth counter to 100:
# > python3 inject_key_bin.py --key key.der --ctr 100
#
# key will not be modified if --key parameter is not present
# counter will not be modified if --ctr parameter is not present

from __future__ import print_function
from asn1crypto.keys import ECPrivateKey
import hashlib
import argparse
import sys
import struct
import os

parser = argparse.ArgumentParser()
parser.add_argument("--bin", default="build/u2f.bin",
                    help='.bin file to inject keys into. Or "stdin"')
parser.add_argument("--key", help="EC private key in DER format")
parser.add_argument("--ctr", default=0, type=int, help="Value of auth counter")
parser.add_argument("--offset", default=0xB400, type=int, help="Offset within file to patch")
args = parser.parse_args()

fname, fext = os.path.splitext(args.bin)
assert fext == ".bin"

print("Target binary file:", args.bin)
print("Key injection offset 0x%0X" % args.offset)

# load and parse private key
if not args.key:
    print("Key not modified")
else:
    print("Injecting key from", args.key)
    if args.key == "stdin":
        stdin = sys.stdin.buffer if hasattr(sys.stdin, "buffer") else sys.stdin
        der = stdin.read()
    else:
        with open(args.key, "rb") as f:
            der = f.read()
    key = ECPrivateKey.load(der)

    # convert key into raw bytes and calculate it's sha256
    key_bytes = bytearray.fromhex(format(key["private_key"].native, '064x'))
    key_hash = hashlib.sha256(key_bytes).digest()
    # pad key to 1KiB
    key_blob = (key_bytes + key_hash).ljust(1024, b"\x00")
    assert len(key_blob) == 1024

    with open(args.bin, 'r+b') as f:
        f.seek(args.offset)
        f.write(key_blob)

if not args.ctr:
    print("Counter not modified")
else:
    print("Injecting counter", args.ctr)
    # fill authentication counter
    ctr_blob = struct.pack("<I", args.ctr) * 256

    with open(args.bin, 'r+b') as f:
        f.seek(args.offset + 1024)
        f.write(ctr_blob)
