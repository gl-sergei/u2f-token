#!/bin/bash

set -e

cat > opnssl.cnf <<EOF
[req]
distinguished_name = req_distinguished_name
[req_distinguished_name]
EOF

# generate key and self-signed certificate
openssl ecparam -genkey -name prime256v1 -out attestation_key.pem
openssl req -config opnssl.cnf -x509 -sha256 -nodes -days 3650 -key attestation_key.pem -subj "/CN=U2F Token" -out attestation.pem

# convert to der
openssl x509 -outform der -in attestation.pem -out attestation.der
openssl ec -in attestation_key.pem -outform der -out attestation_key.der

# generate C code
python dump-der.py > certificates.c
