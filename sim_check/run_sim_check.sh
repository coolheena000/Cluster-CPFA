#!/bin/bash
# Runs Carlos's baseline CPFA in ARGoS without the GUI, on all three food distributions.
# Usage: ./sim_check/run_sim_check.sh [runs_per_distribution]   (default 3)
# Each run takes about 3 seconds. Output: food collected per run, and the average.

RUNS=${1:-3}
HERE="$(cd "$(dirname "$0")" && pwd)"
CARLOS="$HERE/../../Cluster-CPFA"   # library paths in the XML are relative to this folder
TMP="$(mktemp -d)"

cd "$CARLOS" || { echo "Cluster-CPFA folder not found at $CARLOS"; exit 1; }

for d in random powerlaw clustered; do
    total=0
    for s in $(seq 1 "$RUNS"); do
        sed "s/SEED/$s/" "$HERE/cpfa_baseline_$d.xml" > "$TMP/run.xml"
        food=$(argos3 -c "$TMP/run.xml" 2>&1 | grep -E '^[0-9.]+, [0-9.]+$' | awk -F', ' '{printf "%d", $2}')
        echo "$d seed $s: $food food"
        total=$((total + food))
    done
    echo ">> $d average: $(echo "scale=1; $total / $RUNS" | bc)"
    echo
done
rm -rf "$TMP"
