#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# dump-der.py - convert DER encoded certificate and EC key into C header
#
# Copyright (C) 2017-2019 Sergei Glushchenko
# Author: Sergei Glushchenko <gl.sergei@gmail.com>
#
# This file is a part of U2F firmware for STM32 and EFM32HG
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# As additional permission under GNU GPL version 3 section 7, you may
# distribute non-source form of the Program without the copy of the
# GNU GPL normally required by section 4, provided you inform the
# recipients of GNU GPL by a written offer.

from __future__ import print_function
from asn1crypto.keys import ECPrivateKey

attestation_cert_def = '''const struct attestation_cert  __attribute__ ((section(".attestation.cert"))) attestation_cert = {
  .der_len = ATTESTATION_DER_LEN,
  .der = attestation_der,
  .key = attestation_key
};'''

def pk_to_c_array(name, pk_der):
    # parse der format
    pk = ECPrivateKey.load(pk_der)

    # extract private key
    pk_native = pk['private_key'].native

    # translate to hex string
    pk_hex = format(pk_native, '064x')

    # split by pairs of characters
    hex_bytes = ["0x" + pk_hex[i:i + 2] for i in range(0, len(pk_hex), 2)]

    # make string C array declaration
    return "const uint8_t " + name + "[32] = {" + ", ".join(hex_bytes) + "};"

def cert_to_c_array(name, der):
    defname = name.upper() + "_LEN"
    if hasattr(der, 'hex'):
        hex_str = der.hex()
    else:
        hex_str = der.encode('hex')
    hex_bytes = ["0x" + hex_str[i:i + 2] for i in range(0, len(hex_str), 2)]

    define = "#define " + defname + " " + str(len(der))
    array = "const uint8_t " + name + "[" + defname + "] = {" + ", ".join(hex_bytes) + "};"
    return define + "\n" + array

with open("attestation.der", "rb") as f:
    print(cert_to_c_array("attestation_der", f.read()))

with open("attestation_key.der", "rb") as f:
    print(pk_to_c_array("attestation_key", f.read()))

print(attestation_cert_def)
