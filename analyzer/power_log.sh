#!/bin/bash
# Logs Raspberry Pi supply undervoltage once a second, on screen and to ~/power_<label>_<date>_<time>.csv.
# Usage: analyzer/power_log.sh [label]     Ctrl-C to stop.
label=${1:-test}
out=~/power_${label}_$(date +%Y%m%d_%H%M%S).csv

if [ -t 1 ]; then
    red=$'\e[1;31m'; yel=$'\e[33m'; grn=$'\e[32m'; off=$'\e[0m'
fi

read_throttled() {
    raw=$(vcgencmd get_throttled 2>&1); th=${raw#*=}
    [[ $th =~ ^0x[0-9a-fA-F]+$ ]]
}

if ! read_throttled; then
    echo "power_log.sh: bad vcgencmd get_throttled reading: '$raw'" >&2
    exit 1
fi

if ! echo "time,undervoltage,undervoltage_since_boot" > "$out"; then
    echo "power_log.sh: cannot create $out" >&2
    exit 1
fi
echo "Power log '$label' (Ctrl-C to stop). CSV: $out"

while sleep 1; do
    now=$(date +%T)
    if ! read_throttled; then
        echo "$now,," >> "$out"
        echo "$now  ${red}reading failed: '$raw'${off}"
        continue
    fi
    uv=$(( th & 0x1 ))
    uv_boot=$(( (th >> 16) & 0x1 ))
    echo "$now,$uv,$uv_boot" >> "$out"

    if (( uv )); then
        status="${red}UNDERVOLTAGE${off}"
    else
        status="${grn}power ok${off}"
    fi
    if (( uv_boot )); then
        status+="  ${yel}(undervoltage seen since boot)${off}"
    fi
    echo "$now  $status"
done
