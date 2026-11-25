#!/bin/bash
# One robot drives from the nest to the tag 13 corner and back, using the
# overhead camera. Joins NETGEAR15, finds the robot by MAC, then runs it.
#
# Run:  ./robot_goto13.sh 05367                  (to the food closest to tag 13)
#       ./robot_goto13.sh 05367 corner           (to the tag 13 corner only)
#       ./robot_goto13.sh 05367 corner 0.749 0.749  (with arena half sizes in metres)
# Robots with tags: 05367 = tag 1, 05333 = tag 2, 05359 = tag 3, 05417 = tag 4
# Stop: Ctrl+C or Q in the camera window

cd "$(dirname "$0")"

ROBOT="$1"
case "$ROBOT" in
    05367|5367) MAC="cc:50:e3:b7:fe:bc"; LAST_IP="192.168.1.2"; TAG=1 ;;
    05333|5333) MAC="cc:50:e3:b8:5:f8";  LAST_IP="192.168.1.4"; TAG=2 ;;
    05359|5359) MAC="cc:50:e3:b7:fe:6c"; LAST_IP="192.168.1.7"; TAG=3 ;;
    05417|5417) MAC="cc:50:e3:a1:f8:58"; LAST_IP="192.168.1.5"; TAG=4 ;;
    *) echo "Usage: ./robot_goto13.sh ROBOT_NUMBER   (05367 05333 05359 05417)"; exit 1 ;;
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

# 3. Build (if needed) and run
if [ ! -x ./test_goto_tag13 ] || [ test_goto_tag13.cpp -nt test_goto_tag13 ] || [ EpuckHardware.cpp -nt test_goto_tag13 ]; then
    export PKG_CONFIG_PATH=$(brew --prefix opencv)/lib/pkgconfig
    clang++ -std=c++17 test_goto_tag13.cpp EpuckHardware.cpp $(pkg-config --cflags --libs opencv5) -o test_goto_tag13 || exit 1
fi
echo "Robot $ROBOT carries tag $TAG"
shift
./test_goto_tag13 "$IP" "$TAG" "$@"
