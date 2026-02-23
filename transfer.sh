TARGET_IP_ADDRESS=192.168.1.9
SSHPASS="qnxuser"
ROOTPASS="root"

# Remove files as root
sshpass -p $ROOTPASS ssh root@$TARGET_IP_ADDRESS 'rm -rf /data/home/qnxuser/opt/ros/nodes/*'

# Copy files as qnxuser
sshpass -p $SSHPASS scp -rv ./install/aarch64le/* qnxuser@$TARGET_IP_ADDRESS:/data/home/qnxuser/opt/ros/nodes

if [ $? -eq 0 ]; then
    echo "Copying Complete."
else
    echo "Copying FAILED."
fi
