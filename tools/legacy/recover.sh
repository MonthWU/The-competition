#!/bin/bash
# 恢复环境脚本

packages=(
    "python3-colcon-common-extensions"
    "python3-pyzbar"
)
repos=(
    "gitea@192.168.137.250:neolux/GongzongAppli.git"
)
ws="dev_ws1"

echo "Recovering network configuration"
# cat > /etc/NetworkManager/system-connections/Neolux\ Lee.nmconnection <<EOF
# ...
# EOF
# chmod 600 /etc/NetworkManager/system-connections/Neolux\ Lee.nmconnection
systemctl restart NetworkManager

echo "Recovering environment"
echo "----- Neolux Lee -----"

echo "Updating apt package list"
# sudo apt-get update -y
if [ $? -ne 0 ]; then
    echo "Failed to update apt package list"
    exit 1
fi


echo "Packages to install: $packages"
# sudo apt-get install -y $packages
if [ $? -ne 0 ]; then
    echo "Failed to install packages"
    exit 1
fi

echo "Recovering SSH keys"
mkdir -pv ~/.ssh
cat > ~/.ssh/id_rsa <<EOF
...
EOF
chmod 600 ~/.ssh/id_rsa
cat > ~/.ssh/id_rsa.pub <<EOF
...
EOF
chmod 644 ~/.ssh/id_rsa.pub
cat > ~/.ssh/known_hosts <<EOF
...
EOF
chmod 644 ~/.ssh/known_hosts

echo "Recovering development workspace"
mkdir -pv ~/$ws

for repo in ${repos[@]}; do
    echo "Cloning $repo into ~/$ws/$(basename $repo .git)"
    git clone $repo ~/$ws/$(basename $repo .git)
    if [ $? -ne 0 ]; then
        echo "Failed to clone $repo"
        exit 1
    fi
done

echo "Done!"