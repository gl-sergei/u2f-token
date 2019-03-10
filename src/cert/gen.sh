#!/usr/bin/env bash

set -e

cat > opnssl.cnf <<EOF

[req]
x509_extensions = usr_cert

[usr_cert]
1.3.6.1.4.1.45724.2.1.1=ASN1:FORMAT:BITLIST,BITSTRING:2

EOF

# generate key and self-signed certificate
openssl ecparam -genkey -name prime256v1 -out attestation_key.pem
openssl req -new -sha256 -key attestation_key.pem -out csr.csr -subj "/C=US/CN=U2F Token"
openssl req -config opnssl.cnf -x509 -sha256 -days 3650 -key attestation_key.pem -in csr.csr -out attestation.pem

# convert to der
openssl x509 -outform der -in attestation.pem -out attestation.der
openssl ec -in attestation_key.pem -outform der -out attestation_key.der
