#!/bin/bash
# Send a Zephyr image to the running board over the network: the app's /update resource
# (module/lib/fw_update) writes it to the loader slot that is not running and reboots
# into it; this waits for the board's web server to come back. The loader's loader.bin
# goes the same way and replaces the loader, without a reboot.
#   ./update.sh build/scope/zephyr/zephyr.bin [address]
#   ./update.sh build/loader/loader.bin [address]
set -e
bin=$1
address=${2:-192.168.1.207}
[ -f "$bin" ] || { echo "usage: $0 zephyr.bin|loader.bin [address]" >&2; exit 1; }

echo "slots before: $(curl -sS --max-time 10 "http://$address/update")"
echo "sending $(stat -c %s "$bin") bytes to $address"
start=$(date +%s.%N)
# no "Expect: 100-continue", the server would make curl wait for it
reply=$(curl -sS --fail-with-body --max-time 120 -H 'Expect:' \
    -H 'Content-Type: application/octet-stream' --data-binary @"$bin" "http://$address/update")
sent=$(date +%s.%N)
echo "$reply"
printf 'written in %.1f s\n' "$(echo "$sent - $start" | bc)"
if [[ "$reply" != *'"rebooting"'* ]]; then
    exit 0
fi

echo "waiting for the reboot"
sleep 2
for _ in $(seq 60); do
    if curl -s -o /dev/null --max-time 2 "http://$address/update"; then
        printf 'back up %.1f s after the upload\n' "$(echo "$(date +%s.%N) - $sent" | bc)"
        echo "slots after: $(curl -sS --max-time 10 "http://$address/update")"
        exit 0
    fi
    sleep 1
done
echo "no answer from $address after a minute" >&2
exit 1
