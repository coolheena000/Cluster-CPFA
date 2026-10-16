#!/bin/bash
# Same flash as program_manual_conf.sh, but at 115200 baud (no baud change)
# and a fixed 4MB flash size: the 230400 baud switch times out on this setup.
# Usage: ./flash_slow.sh          (robot on, connected by USB)

cd "$(dirname "$0")"
port=/dev/cu.usbmodemEPUCK3
python=/opt/miniconda3/envs/cpfa-robot/bin/python

for f in bootloader.bin ESP32_E-Puck_2.bin partitions_singleapp.bin; do
    if [ ! -f "$f" ]; then
        echo "Cannot find $f"
        exit 1
    fi
done

if [ ! -e "$port" ]; then
    echo "Robot not detected on $port, check the USB cable and turn the robot on"
    exit 1
fi

"$python" esptool.py --chip esp32 --port "$port" --baud 115200 \
    --before default_reset --after hard_reset \
    write_flash -z --flash_mode dio --flash_freq 40m --flash_size 4MB \
    0x1000 bootloader.bin 0x10000 ESP32_E-Puck_2.bin 0x8000 partitions_singleapp.bin
