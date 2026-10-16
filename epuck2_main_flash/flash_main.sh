#!/bin/bash
# Flash GCtronic FACTORY firmware to the e-puck2 MAIN processor (STM32).
# Needed for Wi-Fi mode (selector 15). Same GDB commands as GCtronic's program.sh.
# Robot on USB and switched ON. Port EPUCK1 = GDB server (main processor).
cd "$(dirname "$0")"
port=/dev/cu.usbmodemEPUCK1
if [ ! -e "$port" ]; then
    echo "Robot not detected on $port - connect USB and switch the robot on"
    exit 1
fi
echo "Flashing factory firmware to main processor via $port ..."
./arm-none-eabi-gdb --interpreter=mi -nx \
    -ex "target extended-remote $port" \
    -ex "monitor swdp_scan" \
    -ex "attach 1" \
    -ex "set mem inaccessible-by-default off" \
    -ex "load" \
    e-puck2_main-processor_05.02.26_3f17433.elf \
    -ex "set confirm off" \
    -ex quit
