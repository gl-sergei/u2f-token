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
# > python3 inject_key_bin.py --key key.der --ctr 100 --bin build/u2f.bin
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

# TODO: detect correct offset from .bin file.
stm32f103_offset = 0xB800  # See stm32f103.ld
#efm32hg_offset = ????

parser = argparse.ArgumentParser()
parser.add_argument("--bin", default="build/u2f.bin",
                    help='.bin file to inject keys into. Or "stdin"')
parser.add_argument("--key", help="EC private key in DER format")
parser.add_argument("--ctr", default=0, type=lambda x: int(x,0), help="Value of auth counter")
parser.add_argument("--offset", default=stm32f103_offset, type=lambda x: int(x,0), help="Offset within file to patch")
args = parser.parse_args()

fname, fext = os.path.splitext(args.bin)
assert fext == ".bin"

print("Target binary file:", args.bin)

# load and parse private key
if not args.key:
    print("Key not modified")
else:
    key_offset = args.offset
    print("Injecting key from %s at 0x%0X", (args.key, key_offset)
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
        f.seek(key_offset)
        f.write(key_blob)

if not args.ctr:
    print("Counter not modified")
else:
    ctr_offset = args.offset + 0x400
    print("Injecting counter %d at 0x%0X", (args.ctr, ctr_offset)
    # fill authentication counter
    ctr_blob = struct.pack("<I", args.ctr) * 256

    with open(args.bin, 'r+b') as f:
        f.seek(ctr_offset)
        f.write(ctr_blob)
