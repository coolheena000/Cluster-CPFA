#!/bin/bash
# Wheel test for one robot (no camera): joins NETGEAR15, finds the robot by its
# MAC address, then runs FORWARD, STOP, LEFT, STOP, RIGHT, STOP, BACK, STOP.
# Use it to see how the wheels move on the table surface.
#
# Run:  ./robot_motor.sh 05367            (1 s per step, speed 200)
#       ./robot_motor.sh 05367 2 300      (2 s per step, speed 300 as in CPFA)
# Stop: Ctrl+C (robot gets STOP). Robot stops itself if something is close in front.
# Robots: 05367 05840 05359 05297 05333 05417

cd "$(dirname "$0")"

ROBOT="$1"
case "$ROBOT" in
    05367|5367) MAC="cc:50:e3:b7:fe:bc"; LAST_IP="192.168.1.2" ;;
    05840|5840) MAC="cc:50:e3:b7:fe:68"; LAST_IP="192.168.1.102" ;;
    05359|5359) MAC="cc:50:e3:b7:fe:6c"; LAST_IP="192.168.1.7" ;;
    05297|5297) MAC="cc:50:e3:b1:4a:1c"; LAST_IP="192.168.1.8" ;;
    05333|5333) MAC="cc:50:e3:b8:5:f8";  LAST_IP="192.168.1.4" ;;
    05417|5417) MAC="cc:50:e3:a1:f8:58"; LAST_IP="192.168.1.5" ;;
    *) echo "Usage: ./robot_motor.sh ROBOT_NUMBER   (05367 05840 05359 05297 05333 05417)"; exit 1 ;;
esac

# 1. Mac on NETGEAR15
SSID=$(system_profiler SPAirPortDataType 2>/dev/null | sed -n '/Current Network Information/{n;p;}' | head -1 | sed -E 's/^ +//; s/:$//')
if [ "$SSID" != "NETGEAR15" ]; then
    echo "Joining NETGEAR15 ..."
    for try in 1 2 3; do
        networksetup -setairportnetwork en0 NETGEAR15 ROUTER_PASSWORD >/dev/null 2>&1
        sleep 6
        case "$(ipconfig getifaddr en0)" in 192.168.1.*) break ;; esac
    done
fi
MYIP=$(ipconfig getifaddr en0)
case "$MYIP" in
    192.168.1.*) echo "Mac on NETGEAR15 ($MYIP)" ;;
    *) echo "Could not join NETGEAR15 (Mac IP: ${MYIP:-none}). Join it from the Wi-Fi menu and run again."; exit 1 ;;
esac

# 2. Find the robot by MAC address
echo "Looking for robot $ROBOT ..."
IP=""
for round in 1 2 3; do
    for i in $(seq 2 30); do ping -c 2 -i 0.3 -t 2 "192.168.1.$i" >/dev/null 2>&1 & done
    wait
    IP=$(arp -an | grep -i " at $MAC " | sed -E 's/.*\(([0-9.]+)\).*/\1/' | head -1)
    [ -n "$IP" ] && break
done
if [ -z "$IP" ]; then
    # Robots in Wi-Fi power-save often ignore ping; try the last known IP.
    echo "No ping answer from $ROBOT; trying its last known IP $LAST_IP"
    echo "(if this fails, check the router's Attached Devices page for its IP)"
    IP="$LAST_IP"
fi
echo "Robot $ROBOT is at $IP"

# 3. Wheel test
if [ ! -x ./test_motor_move ] || [ test_motor_move.cpp -nt test_motor_move ] || [ EpuckHardware.cpp -nt test_motor_move ]; then
    clang++ -std=c++17 test_motor_move.cpp EpuckHardware.cpp -o test_motor_move || exit 1
fi
shift
./test_motor_move "$IP" "$@"
