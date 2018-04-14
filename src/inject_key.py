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
# > python3 inject_key.py --key key.der --ctr 100
#

from __future__ import print_function
from asn1crypto.keys import ECPrivateKey
import hashlib
import argparse
import sys
import struct
import os
import tempfile
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument("--elf", default="build/u2f.elf",
                    help=".elf file to inject keys into")
parser.add_argument("--key", help="EC private key in DER format")
parser.add_argument("--ctr", default=1, type=int, help="value of auth counter")
args = parser.parse_args()

# load and parse private key
if args.key:
    with open(args.key, "rb") as f:
        der = f.read()
else:
    stdin = sys.stdin.buffer if hasattr(sys.stdin, "buffer") else sys.stdin
    der = stdin.read()
key = ECPrivateKey.load(der)

# convert key into raw bytes and calculate it's sha256
key_bytes = bytearray.fromhex(format(key["private_key"].native, '064x'))
key_hash = hashlib.sha256(key_bytes).digest()

# fill authentication counter
ctr_bytes = struct.pack("<I", args.ctr) * 256

# pad key and append ctr to produce 2k output blob
blob = (key_bytes + key_hash).ljust(1024, b"\x00") + ctr_bytes

assert len(blob) == 2048
fname, fext = os.path.splitext(args.elf)
assert fext == ".elf"

with tempfile.NamedTemporaryFile(delete=True) as f:
    f.write(blob)
    f.flush()
    # replace contents of .flash_storage section with desired key and counter
    ret = subprocess.call(["arm-none-eabi-objcopy", "--update-section",
                           ".flash_storage=" + f.name, args.elf])
    if ret != 0:
        raise Exception("Failed to patch .elf file!")
    # generate binary for bootloader
    ret = subprocess.call(["arm-none-eabi-objcopy", "-O", "binary",
                           args.elf, fname + ".bin"])
    if ret != 0:
        raise Exception("Failed to create .bin file!")
