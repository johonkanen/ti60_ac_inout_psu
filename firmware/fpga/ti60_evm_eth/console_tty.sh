#!/bin/bash
# Prints the kit's console tty (FT4232H interface 2). The ttyUSB number changes
# whenever the kit re-enumerates.
for t in /dev/ttyUSB*; do
    p=$(udevadm info -q property "$t" 2>/dev/null)
    if grep -q 'ID_MODEL=Titanium_Ti60F225_Development_Kit' <<<"$p" && grep -q 'ID_USB_INTERFACE_NUM=02' <<<"$p"; then
        echo "$t"; exit 0
    fi
done
exit 1
