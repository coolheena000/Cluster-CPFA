#!/bin/bash
# One command for a full CPFA run on the table: joins NETGEAR15, finds the
# robots by MAC address (IPs can change), then runs run_four_robots, which
# first checks the camera calibration (corner tags 10-13) and then starts CPFA.
#
# Run:  ./run_cpfa.sh 05367 05333
# Robots with tags: 05367 = tag 1, 05333 = tag 2, 05359 = tag 3, 05417 = tag 4
# Stop: Ctrl+C or Q in the camera window

cd "$(dirname "$0")"

if [ $# -eq 0 ]; then
    echo "Usage: ./run_cpfa.sh 05367 05333   (robots: 05367 05333 05359 05417)"
    exit 1
fi

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

# 2. Wake all robots (ping the subnet), then find each one by MAC
echo "Looking for robots ..."
for round in 1 2 3; do
    for i in $(seq 2 30); do ping -c 2 -i 0.3 -t 2 "192.168.1.$i" >/dev/null 2>&1 & done
    wait
done

SLOT=("-" "-" "-" "-")
for ROBOT in "$@"; do
    case "$ROBOT" in
        05367|5367) N=0; MAC="cc:50:e3:b7:fe:bc"; LAST_IP="192.168.1.2" ;;
        05333|5333) N=1; MAC="cc:50:e3:b8:5:f8";  LAST_IP="192.168.1.4" ;;
        05359|5359) N=2; MAC="cc:50:e3:b7:fe:6c"; LAST_IP="192.168.1.7" ;;
        05417|5417) N=3; MAC="cc:50:e3:a1:f8:58"; LAST_IP="192.168.1.5" ;;
        *) echo "Robot $ROBOT has no tag. Use 05367 05333 05359 05417"; exit 1 ;;
    esac
    IP=$(arp -an | grep -i " at $MAC " | sed -E 's/.*\(([0-9.]+)\).*/\1/' | head -1)
    if [ -z "$IP" ]; then
        echo "No ping answer from $ROBOT; trying its last known IP $LAST_IP"
        IP="$LAST_IP"
    fi
    echo "Robot $ROBOT (tag $((N + 1))) is at $IP"
    SLOT[$N]="$IP"
done

# 3. Build (if needed) and run CPFA
if [ ! -x ./run_four_robots ] || [ run_four_robots.cpp -nt run_four_robots ]; then
    export PKG_CONFIG_PATH=$(brew --prefix opencv)/lib/pkgconfig
    clang++ -std=c++17 run_four_robots.cpp EpuckHardware.cpp $(pkg-config --cflags --libs opencv5) -o run_four_robots || exit 1
fi
echo "./run_four_robots ${SLOT[*]}"
./run_four_robots "${SLOT[@]}"
