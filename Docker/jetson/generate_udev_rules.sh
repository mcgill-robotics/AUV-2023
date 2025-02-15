echo "Enter desired device name (i.e. /dev/____): "
read dev_name
echo "Enter current device path (i.e. /dev/____):"
read dev_path

if [[ ! -e "/dev/$dev_path" ]]; then
    echo "ERROR: Device file /dev/$dev_path does not exist."
    exit 1
fi

CMD_OUTPUT=$(udevadm info -a -p $(udevadm info -q path -n $dev_path))
KERNEL=$(echo $CMD_OUTPUT | grep -oP 'KERNEL=="\K[^"]+' | head -n 1 | sed 's/[0-9]*$//')
SUBSYSTEM=$(echo $CMD_OUTPUT | grep -oP 'SUBSYSTEM=="\K[^"]+' | head -n 1)
IDVENDOR=$(echo $CMD_OUTPUT | grep -oP 'idVendor}=="\K[^"]+' | head -n 1)
IDPRODUCT=$(echo $CMD_OUTPUT | grep -oP 'idProduct}=="\K[^"]+' | head -n 1)
SERIALNO=$(echo $CMD_OUTPUT | grep -oP 'serial}=="\K[^"]+' | head -n 1)

if [[ -e "$dev_name.rules" ]]; then
    echo "WARN: Rules file $dev_name.rules already exists. Overwrite? [Y/n] "
    read tmp
    if [[ "$tmp" == "y" || "$tmp" == "Y" ]]; then
        if [[ $SUBSYSTEM == "video4linux" ]]; then
            sudo echo "SUBSYSTEM==\"$SUBSYSTEM\", KERNEL==\"$KERNEL[0-9]*\", ATTRS{idVendor}==\"$IDVENDOR\", ATTRS{idProduct}==\"$IDPRODUCT\", SYMLINK+=\"$dev_name\"" > $dev_name.rules
        else
            sudo echo "SUBSYSTEM==\"$SUBSYSTEM\", KERNEL==\"$KERNEL[0-9]*\", ATTRS{idVendor}==\"$IDVENDOR\", ATTRS{idProduct}==\"$IDPRODUCT\", ATTRS{serial}==\"$SERIALNO\", SYMLINK+=\"$dev_name\"" > $dev_name.rules
        fi
    fi
else
    if [[ $SUBSYSTEM == "video4linux" ]]; then
        sudo echo "SUBSYSTEM==\"$SUBSYSTEM\", KERNEL==\"$KERNEL[0-9]*\", ATTRS{idVendor}==\"$IDVENDOR\", ATTRS{idProduct}==\"$IDPRODUCT\", SYMLINK+=\"$dev_name\"" > $dev_name.rules
    else
        sudo echo "SUBSYSTEM==\"$SUBSYSTEM\", KERNEL==\"$KERNEL[0-9]*\", ATTRS{idVendor}==\"$IDVENDOR\", ATTRS{idProduct}==\"$IDPRODUCT\", ATTRS{serial}==\"$SERIALNO\", SYMLINK+=\"$dev_name\"" > $dev_name.rules
    fi
fi

echo "Copy $dev_name.rules file to /etc/udev/rules.d (requires sudo)? [Y/n] "
read tmp
if [[ "$tmp" == "y" || "$tmp" == "Y" ]]; then
    sudo cp $dev_name.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
fi
