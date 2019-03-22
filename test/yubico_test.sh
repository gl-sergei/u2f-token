# Requires dfu-utils to be installed.
# This runs through the steps described in https://github.com/Yubico/libu2f-host documentation
set -x
curl 'https://demo.yubico.com/wsapi/u2f/enroll?username=tomu&password=test' > regchallenge.json
cat regchallenge.json
echo "Touch key to confirm registration"
u2f-host -a register -o 'https://demo.yubico.com'  < regchallenge.json > regdata.json
#cat regdata.json
curl https://demo.yubico.com/wsapi/u2f/bind -d "username=tomu&password=test&data=$(cat regdata.json)"
curl 'https://demo.yubico.com/wsapi/u2f/sign?username=tomu&password=test' > challenge.json
cat challenge.json
echo "Touch key to confirm authentication"
u2f-host -a authenticate -o 'https://demo.yubico.com'  < challenge.json > signature.json
#cat signature.json
curl https://demo.yubico.com/wsapi/u2f/verify -d "username=tomu&password=test&data=$(cat signature.json)" > response.json
cat response.json
echo ""

