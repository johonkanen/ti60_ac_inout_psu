#!/bin/bash
# Put the web client of web/ into the board's flash: each file gzipped to <name>.gz in
# the littlefs that the scope app serves, through its /files/ resource. Files of the
# board that web/ no longer has are removed, except the collection page's scaling.json and the
# waveform page's loop_response.json.
#   ./upload_web.sh [address]
set -e
address=${1:-192.168.1.207}
cd "$(dirname "$0")/web"
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT

wanted=()
for file in *; do
    case "$file" in LICENSE*) continue ;; esac
    gzip -9 -n -c "$file" > "$tmp/$file.gz"
    curl -sS --fail-with-body -H 'Expect:' --data-binary @"$tmp/$file.gz" \
        "http://$address/files/$file.gz"
    wanted+=("$file.gz")
done

for name in $(curl -sS --fail "http://$address/files/" | python3 -c \
        'import json, sys; print(" ".join(f["name"] for f in json.load(sys.stdin)["files"]))'); do
    # the data collection page's scaling and the waveform page's measured loop response are
    # data, not part of the web client
    if [[ ! " ${wanted[*]} " == *" $name "* && $name != scaling.json.gz && $name != loop_response.json.gz ]]; then
        curl -sS --fail-with-body -X DELETE "http://$address/files/$name"
    fi
done
curl -sS "http://$address/files/"
