#!/bin/bash
# Prints the board's console tty: channel C of the FT4232H MiniModule (serial FT80TJL7).
# The ttyUSB number changes whenever the module re-enumerates.
for t in /dev/ttyUSB*; do
    p=$(udevadm info -q property "$t" 2>/dev/null)
    if grep -q 'ID_SERIAL_SHORT=FT80TJL7' <<<"$p" && grep -q 'ID_USB_INTERFACE_NUM=02' <<<"$p"; then
        echo "$t"; exit 0
    fi
done
exit 1
