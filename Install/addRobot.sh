#!/bin/bash

set -e
set -o pipefail

if [ "$#" -lt 4 ]; then
    echo "Illegal number of parameters"
    echo "Usage: $0 <IP address> <Name> <Robot ID> <Team ID>"
    exit 1
fi

# parameters
IP_ADDRESS="$1"
ROBOT_NAME="$2"
ID="$3"
TEAM="$4"

scriptPath=$(echo $0 | sed "s|^\.\./|`pwd`/../|" | sed "s|^\./|`pwd`/|")
basePath=$(dirname "${scriptPath}")
currPath=$(pwd)
keySource=../Config/Keys/id_rsa_nao
keyFile=/tmp/id_rsa_nao
sshoptions="-i $keyFile -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=quiet -o ConnectTimeout=5 -o ControlMaster=auto -o ControlPath=~/.ssh/copyfiles-%r@%h:%p -o ControlPersist=60"
cd "${basePath}"
cp $keySource $keyFile
chmod 600 $keyFile

if ssh $sshoptions root@$IP_ADDRESS '[ "$(hostname)" != "Nao" ]'; then
    echo "Already configured!";
    
    read -p "Continue? (y/N) " yn
    case $yn in
        [Yy]* )
            ;;
        * ) exit 1;;
    esac
fi

HEAD_ID=$( ssh $sshoptions root@$IP_ADDRESS 'cat /sys/qi/head_id' )
echo "HEAD_ID: $HEAD_ID"

BODY_ID=$( ssh $sshoptions root@$IP_ADDRESS egrep -Eom1 'P[0-9][0-9A-Z]+'  /home/nao/.config/hal/Device_Body_dump.xml )
echo "BODY_ID: $BODY_ID"

ssh $sshoptions root@$IP_ADDRESS "echo '$ROBOT_NAME' > /etc/hostname"
ssh $sshoptions root@$IP_ADDRESS "hostname '$ROBOT_NAME'"
ssh $sshoptions root@$IP_ADDRESS "sed -i 's#127\.0\.0\.1\W.*#127.0.0.1\tlocalhost $ROBOT_NAME#' /etc/hosts"

sed -i "/];/i \ \ { name = $ROBOT_NAME; headId = $HEAD_ID; bodyId = $BODY_ID; id = $ID; lan = 10.1.$TEAM.$ID; wlan = 10.0.$TEAM.$ID; naoVersion = V6; }," ../Config/Robots/robots.cfg

# disable ssh password login
ssh $sshoptions root@$IP_ADDRESS "sed -i 's!#PasswordAuthentication yes!PasswordAuthentication no!' /etc/ssh/sshd_config"

# copy public ssh key
ssh $sshoptions root@$IP_ADDRESS "mkdir -p /home/nao/.ssh"
ssh $sshoptions root@$IP_ADDRESS "mkdir -p /root/.ssh"
cat "../Config/Keys/id_rsa_nao.pub" | ssh $sshoptions root@$IP_ADDRESS "cat - >> /home/nao/.ssh/authorized_keys"
cat "../Config/Keys/id_rsa_nao.pub" | ssh $sshoptions root@$IP_ADDRESS "cat - >> /root/.ssh/authorized_keys"
ssh $sshoptions root@$IP_ADDRESS "chmod 700 /home/nao/.ssh /root/.ssh"
ssh $sshoptions root@$IP_ADDRESS "chmod 600 /home/nao/.ssh/authorized_keys /root/.ssh/authorized_keys"
ssh $sshoptions root@$IP_ADDRESS "chown -R 1001:1001 /home/nao/.ssh"

ssh $sshoptions root@$IP_ADDRESS "cat - > /etc/netplan/default.yaml" <<EOT
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      optional: true
      addresses:
        - 10.1.$TEAM.$ID/16
      dhcp4: false #eth0
      dhcp6: false #eth0
EOT

cat - <<EOT > /etc/netplan/wifi.yaml
network:
  version: 2
  renderer: networkd
  wifis:
    wlan0:
      optional: true
      access-points:
        "SPL_5GHz":
          auth:
            key-management: "psk"
            password: "Nao?!Nao?!"
      addresses:
        - 10.0.$TEAM.$ID/16
      dhcp4: false
      dhcp6: false
EOT

ssh $sshoptions root@$IP_ADDRESS "netplan apply </dev/null >/dev/null 2>&1 &"

echo "LAN IP: 10.1.$TEAM.$ID/16"
echo "WLAN IP: 10.0.$TEAM.$ID/16"

echo "Configuration done. Open Dorsh and deploy robot."
