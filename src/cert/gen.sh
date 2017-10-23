# generate key and self-signed certificate
openssl ecparam -genkey -name prime256v1 -out attestation_key.pem
openssl req -new -sha256 -key attestation_key.pem -out csr.csr -subj "/C=TH/ST=Bangkok/L=Bangkok/O=GL Sergei/OU=Dev/CN=GL Sergei/emailAddress=gl.sergei@gmail.com"
openssl req -x509 -sha256 -days 365 -key attestation_key.pem -in csr.csr -out attestation.pem

# convert to der
openssl x509 -outform der -in attestation.pem -out attestation.der
openssl ec -in attestation_key.pem -outform der -out attestation_key.der

# generate C code
python dump-der.py > certificates.c
