# Basic test
# Requires u2f-host to be installed.
# This runs through the steps described in https://github.com/Yubico/libu2f-host documentation
# If run with -k argument, the json files are retainsd.
set -x

user="tomu-$(date +%s)"
pass=$(uuidgen)

curl "https://demo.yubico.com/wsapi/u2f/enroll?username=$user&password=$pass" > regchallenge.json
cat regchallenge.json
echo "Touch key to confirm registration"
u2f-host -a register -o 'https://demo.yubico.com'  < regchallenge.json > regdata.json
#cat regdata.json
curl https://demo.yubico.com/wsapi/u2f/bind -d "username=$user&password=$pass&data=$(cat regdata.json)"
curl "https://demo.yubico.com/wsapi/u2f/sign?username=$user&password=$pass" > challenge.json
cat challenge.json
echo "Touch key to confirm authentication"
u2f-host -a authenticate -o 'https://demo.yubico.com'  < challenge.json > signature.json
#cat signature.json
curl https://demo.yubico.com/wsapi/u2f/verify -d "username=$user&password=$pass&data=$(cat signature.json)" > response.json
cat response.json
echo ""

if [ "$1" !=  "-k" ]
then
  rm *.json
fi

